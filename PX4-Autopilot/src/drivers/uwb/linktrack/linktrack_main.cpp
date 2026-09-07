/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 ****************************************************************************/

/**
 * @file linktrack_main.cpp
 *
 * Driver for Nooploop LinkTrack UWB positioning modules (NLink binary protocol).
 *
 * Parses tag frame 0 (header 0x55, function mark 0x01, 128 bytes, uint8 sum
 * check) and publishes the tag position/velocity as vehicle_visual_odometry
 * for EKF2 external vision fusion (EKF2_EV_*).
 *
 * If the module is configured for NMEA output instead of the NLink binary
 * protocol, this driver cannot parse it - switch the module back to the
 * LinkTrack protocol with the Nooploop LinkTrack Assistant tool (a warning
 * is printed when NMEA-like data is detected).
 *
 * Protocol reference: Nooploop nlink_unpack (nlt_linktrack_tagframe0).
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <lib/parameters/param.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_odometry.h>

#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

using namespace time_literals;

// NLink tag frame 0: fixed size including header, function mark and check sum
static constexpr size_t LT_FRAME_SIZE = 128;
static constexpr uint8_t LT_FRAME_HEADER = 0x55;
static constexpr uint8_t LT_FUNCTION_MARK_TAGFRAME0 = 0x01;

// Fixed-point scale factors of the NLink protocol
static constexpr float LT_SCALE_POS = 1000.0f;  // int24 -> m
static constexpr float LT_SCALE_VEL = 10000.0f; // int24 -> m/s
static constexpr float LT_SCALE_EOP = 100.0f;   // uint8 -> m

extern "C" __EXPORT int linktrack_main(int argc, char *argv[]);

class LinkTrack : public ModuleBase<LinkTrack>, public ModuleParams
{
public:
	LinkTrack(const char *device, int baud);
	~LinkTrack() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static LinkTrack *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status */
	int print_status() override;

	/** @see ModuleBase::run */
	void run() override;

private:

	int open_serial(const char *device, int baud);
	void parse_byte(uint8_t b);
	void handle_frame();
	static int32_t parse_int24(const uint8_t *b) { return ((int32_t)(b[0] << 8 | b[1] << 16 | b[2] << 24)) / 256; }
	static float parse_float(const uint8_t *b) { float f; memcpy(&f, b, sizeof(f)); return f; }

	uORB::Publication<vehicle_odometry_s> _visual_odom_pub{ORB_ID(vehicle_visual_odometry)};

	char _device[32] {};
	int _baud{921600};
	int _fd{-1};

	// frame parser state
	uint8_t _frame_buf[LT_FRAME_SIZE] {};
	size_t _frame_idx{0};
	uint8_t _sync_state{0}; // 0=idle 1=got 0x55 2=collecting

	// statistics
	uint32_t _frame_count{0};
	uint32_t _checksum_errors{0};
	uint32_t _step_rejects{0};
	uint32_t _nmea_count{0};
	hrt_abstime _last_frame_time{0};
	hrt_abstime _last_nmea_warn{0};
	uint8_t _tag_id{0};
	float _voltage{0.0f};

	// glitch rejection state (LT_MAX_STEP)
	float _last_rx_pos[2] {};
	float _last_acc_pos[2] {};
	uint32_t _resync_count{0};
	bool _have_acc_pos{false};

	// first-order low-pass state (LT_LPF_TAU)
	bool _lpf_init{false};
	float _lpf_pos[2] {};
	float _lpf_vel[2] {};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::LT_TF_ROT>) _param_tf_rot,
		(ParamFloat<px4::params::LT_TF_YAW>) _param_tf_yaw,
		(ParamFloat<px4::params::LT_COV_POS>) _param_cov_pos,
		(ParamFloat<px4::params::LT_COV_VEL>) _param_cov_vel,
		(ParamFloat<px4::params::LT_COV_VEL_Z>) _param_cov_vel_z,
		(ParamFloat<px4::params::LT_MAX_STEP>) _param_max_step,
		(ParamFloat<px4::params::LT_LPF_TAU>) _param_lpf_tau
	)
};

LinkTrack::LinkTrack(const char *device, int baud) :
	ModuleParams(nullptr),
	_baud(baud)
{
	strncpy(_device, device, sizeof(_device) - 1);
}

int LinkTrack::open_serial(const char *device, int baud)
{
	int fd = ::open(device, O_RDWR | O_NOCTTY);

	if (fd < 0) {
		PX4_ERR("open %s failed (%d)", device, errno);
		return -1;
	}

	speed_t speed = B921600;

	switch (baud) {
	case 9600: speed = B9600; break;
	case 19200: speed = B19200; break;
	case 38400: speed = B38400; break;
	case 57600: speed = B57600; break;
	case 115200: speed = B115200; break;
	case 230400: speed = B230400; break;
	case 460800: speed = B460800; break;
	case 921600: speed = B921600; break;
	default:
		PX4_WARN("unsupported baud %d, falling back to 921600", baud);
		break;
	}

	struct termios cfg {};

	if (tcgetattr(fd, &cfg) != 0) {
		PX4_ERR("tcgetattr %s failed (%d)", device, errno);
		::close(fd);
		return -1;
	}

	cfmakeraw(&cfg);
	cfsetispeed(&cfg, speed);
	cfsetospeed(&cfg, speed);

	if (tcsetattr(fd, TCSANOW, &cfg) != 0) {
		PX4_ERR("tcsetattr %s failed (%d)", device, errno);
		::close(fd);
		return -1;
	}

	return fd;
}

void LinkTrack::parse_byte(uint8_t b)
{
	switch (_sync_state) {
	case 0:
		if (b == LT_FRAME_HEADER) {
			_frame_buf[0] = b;
			_sync_state = 1;

		} else if (b == '$') {
			// ASCII data: module is configured for NMEA output, not the NLink binary protocol
			_nmea_count++;

			if (hrt_elapsed_time(&_last_nmea_warn) > 10_s) {
				_last_nmea_warn = hrt_absolute_time();
				PX4_WARN("NMEA data detected - switch module to NLink binary protocol (LinkTrack Assistant)");
			}
		}

		break;

	case 1:
		if (b == LT_FUNCTION_MARK_TAGFRAME0) {
			_frame_buf[1] = b;
			_frame_idx = 2;
			_sync_state = 2;

		} else if (b != LT_FRAME_HEADER) {
			_sync_state = 0;
		}

		break;

	case 2:
		_frame_buf[_frame_idx++] = b;

		if (_frame_idx >= LT_FRAME_SIZE) {
			_sync_state = 0;
			handle_frame();
		}

		break;
	}
}

void LinkTrack::handle_frame()
{
	// check sum: uint8 sum over all bytes except the last one
	uint8_t sum = 0;

	for (size_t i = 0; i < LT_FRAME_SIZE - 1; ++i) {
		sum += _frame_buf[i];
	}

	if (sum != _frame_buf[LT_FRAME_SIZE - 1]) {
		_checksum_errors++;
		return;
	}

	const uint8_t *d = _frame_buf;
	// wire layout (nlt_tagframe0_raw_t): [0]=0x55 [1]=0x01 [2]=id [3]=role
	// [4..12] pos int24x3 | [13..21] vel int24x3 | [22..45] dis int24x8
	// [46..57] gyro floatx3 | [58..69] acc floatx3 | [70..81] reserved
	// [82..87] angle int16x3 | [88..103] quaternion floatx4 | [104..107] reserved
	// [108..111] local_time u32 | [112..115] system_time u32 | [116] reserved
	// [117..119] eop u8x3 | [120..121] voltage u16 (mV) | [122..126] reserved | [127] sum
	const uint8_t *pos_raw = &d[4];
	const uint8_t *vel_raw = &d[13];
	const uint8_t *eop_raw = &d[117];

	_tag_id = d[2];
	uint16_t voltage_mv;
	memcpy(&voltage_mv, &d[120], sizeof(voltage_mv));
	_voltage = voltage_mv / 1000.0f;

	float pos_enu[3];
	float vel_enu[3];

	for (int i = 0; i < 3; ++i) {
		pos_enu[i] = parse_int24(&pos_raw[i * 3]) / LT_SCALE_POS;
		vel_enu[i] = parse_int24(&vel_raw[i * 3]) / LT_SCALE_VEL;
	}

	vehicle_odometry_s odom {};
	odom.timestamp = hrt_absolute_time();
	odom.timestamp_sample = odom.timestamp;

	// coordinate mapping: LinkTrack anchor frame (right-handed, Z up) -> NED
	if (_param_tf_rot.get() == 1) {
		// NED passthrough
		odom.position[0] = pos_enu[0];
		odom.position[1] = pos_enu[1];
		odom.position[2] = pos_enu[2];
		odom.velocity[0] = vel_enu[0];
		odom.velocity[1] = vel_enu[1];
		odom.velocity[2] = vel_enu[2];

	} else {
		// ENU -> NED: north = y_enu, east = x_enu, down = -z_enu
		odom.position[0] = pos_enu[1];
		odom.position[1] = pos_enu[0];
		odom.position[2] = -pos_enu[2];
		odom.velocity[0] = vel_enu[1];
		odom.velocity[1] = vel_enu[0];
		odom.velocity[2] = -vel_enu[2];
	}

	// yaw alignment: rotate anchor frame into the EKF local frame (mag-north
	// based). LT_TF_YAW is the heading QGC shows when the vehicle nose points
	// along the anchor X axis.
	const float yaw_offset = math::radians(_param_tf_yaw.get());

	if (fabsf(yaw_offset) > 1e-6f) {
		const float c = cosf(yaw_offset);
		const float s = sinf(yaw_offset);

		const float px = odom.position[0];
		const float py = odom.position[1];
		odom.position[0] = c * px - s * py;
		odom.position[1] = s * px + c * py;

		const float vx = odom.velocity[0];
		const float vy = odom.velocity[1];
		odom.velocity[0] = c * vx - s * vy;
		odom.velocity[1] = s * vx + c * vy;
	}

	odom.pose_frame = vehicle_odometry_s::POSE_FRAME_NED;
	odom.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_NED;

	// covariance: prefer the module's error-of-position estimate (std dev in m),
	// fall back to the configured variance when eop is zero/invalid
	for (int i = 0; i < 3; ++i) {
		const float eop = eop_raw[i] / LT_SCALE_EOP;
		odom.position_variance[i] = (eop > 0.0f) ? (eop * eop) : _param_cov_pos.get();
		odom.velocity_variance[i] = _param_cov_vel.get();
	}

	// vertical velocity from the tag is unreliable (MATH_MODEL2 z is the weak
	// axis); report a huge variance so the EKF effectively ignores it
	odom.velocity_variance[2] = _param_cov_vel_z.get();

	odom.q[0] = NAN;
	odom.q[1] = NAN;
	odom.q[2] = NAN;
	odom.q[3] = NAN;
	odom.angular_velocity[0] = NAN;
	odom.angular_velocity[1] = NAN;
	odom.angular_velocity[2] = NAN;
	odom.orientation_variance[0] = NAN;
	odom.orientation_variance[1] = NAN;
	odom.orientation_variance[2] = NAN;
	odom.reset_counter = 0;

	// glitch rejection (LT_MAX_STEP): drop frames whose horizontal displacement
	// from the last accepted frame is physically impossible (EMI glitches can
	// teleport the solution by meters in one 100 Hz frame). If the new position
	// persists for 10 consecutive self-consistent frames, resync and accept it.
	const float max_step = _param_max_step.get();

	if (max_step > 0.f) {
		const float drx = odom.position[0] - _last_rx_pos[0];
		const float dry = odom.position[1] - _last_rx_pos[1];
		const bool consistent_with_prev = (drx * drx + dry * dry) <= (max_step * max_step);
		_resync_count = consistent_with_prev ? (_resync_count + 1) : 0;

		_last_rx_pos[0] = odom.position[0];
		_last_rx_pos[1] = odom.position[1];

		if (!_have_acc_pos) {
			// first frame ever: accept as reference
			_have_acc_pos = true;
			_last_acc_pos[0] = odom.position[0];
			_last_acc_pos[1] = odom.position[1];

		} else {
			const float dax = odom.position[0] - _last_acc_pos[0];
			const float day = odom.position[1] - _last_acc_pos[1];

			if ((dax * dax + day * day) > (max_step * max_step) && _resync_count < 10) {
				_step_rejects++;
				return;
			}

			_last_acc_pos[0] = odom.position[0];
			_last_acc_pos[1] = odom.position[1];
		}
	}

	// first-order low-pass over horizontal position/velocity (LT_LPF_TAU).
	// Applied after glitch rejection so the EKF receives a smoothed absolute
	// position instead of raw per-frame jitter; 0 disables the filter.
	const float tau = _param_lpf_tau.get();

	if (tau > 0.f) {
		// raw frame interval in seconds (first frame uses a nominal 10 ms period)
		const float raw_dt = (_last_frame_time != 0) ? (odom.timestamp - _last_frame_time) * 1e-6f : 0.01f;
		float dt = raw_dt;

		if (dt < 0.001f) { dt = 0.001f; }
		else if (dt > 0.1f) { dt = 0.1f; }

		// re-lock to the new value if the stream stalled or this is the first frame
		if (!_lpf_init || raw_dt > 0.2f) {
			_lpf_pos[0] = odom.position[0];
			_lpf_pos[1] = odom.position[1];
			_lpf_vel[0] = odom.velocity[0];
			_lpf_vel[1] = odom.velocity[1];
			_lpf_init = true;

		} else {
			const float alpha = dt / (tau + dt);
			_lpf_pos[0] += alpha * (odom.position[0] - _lpf_pos[0]);
			_lpf_pos[1] += alpha * (odom.position[1] - _lpf_pos[1]);
			_lpf_vel[0] += alpha * (odom.velocity[0] - _lpf_vel[0]);
			_lpf_vel[1] += alpha * (odom.velocity[1] - _lpf_vel[1]);
		}

		odom.position[0] = _lpf_pos[0];
		odom.position[1] = _lpf_pos[1];
		odom.velocity[0] = _lpf_vel[0];
		odom.velocity[1] = _lpf_vel[1];
	}

	_visual_odom_pub.publish(odom);

	_frame_count++;
	_last_frame_time = odom.timestamp;
}

void LinkTrack::run()
{
	while (!should_exit()) {
		if (_fd < 0) {
			_fd = open_serial(_device, _baud);

			if (_fd < 0) {
				// retry while the task runs
				px4_usleep(1000000);
				continue;
			}

			PX4_INFO("listening on %s @ %d baud", _device, _baud);
		}

		px4_pollfd_struct_t fds[1];
		fds[0].fd = _fd;
		fds[0].events = POLLIN;

		int ret = px4_poll(fds, 1, 1000);

		if (ret > 0 && (fds[0].revents & POLLIN)) {
			uint8_t buf[256];
			ssize_t n = ::read(_fd, buf, sizeof(buf));

			if (n > 0) {
				for (ssize_t i = 0; i < n; ++i) {
					parse_byte(buf[i]);
				}

			} else if (n < 0 && errno != EAGAIN) {
				PX4_ERR("read %s failed (%d)", _device, errno);
				::close(_fd);
				_fd = -1;
			}
		}
	}

	if (_fd >= 0) {
		::close(_fd);
		_fd = -1;
	}
}

int LinkTrack::print_status()
{
	PX4_INFO("device: %s @ %d baud", _device, _baud);
	PX4_INFO("frames: %lu, checksum errors: %lu, step rejects: %lu", _frame_count, _checksum_errors, _step_rejects);

	if (_nmea_count > 0) {
		PX4_INFO("NMEA bytes seen: %lu (module is in NMEA mode, NLink binary required)", _nmea_count);
	}

	if (_frame_count > 0) {
		PX4_INFO("tag id: %u, voltage: %.2f V, last frame %.1f s ago", _tag_id, (double)_voltage,
			 (double)(hrt_elapsed_time(&_last_frame_time) / 1e6f));

	} else {
		PX4_INFO("no frames received yet");
	}

	return 0;
}

int LinkTrack::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int LinkTrack::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("linktrack",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_SLOW_DRIVER,
				      2048,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

LinkTrack *LinkTrack::instantiate(int argc, char *argv[])
{
	const char *device = "/dev/ttyS3";
	int baud = 921600;

	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:b:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device = myoptarg;
			break;

		case 'b':
			if (strncmp(myoptarg, "p:", 2) == 0) {
				// baud given as parameter name (serial port config mechanism)
				param_t p = param_find(myoptarg + 2);
				int32_t baud_param = 0;

				if (p == PARAM_INVALID || param_get(p, &baud_param) != 0) {
					PX4_ERR("could not read baud param %s", myoptarg + 2);
					return nullptr;
				}

				baud = baud_param;

			} else {
				baud = atoi(myoptarg);
			}

			break;

		default:
			print_usage();
			return nullptr;
		}
	}

	if (baud <= 0) {
		PX4_ERR("invalid baud rate %d", baud);
		return nullptr;
	}

	return new LinkTrack(device, baud);
}

int LinkTrack::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver for Nooploop LinkTrack UWB positioning modules (NLink binary protocol).
Parses tag frame 0 and publishes vehicle_visual_odometry for EKF2 external
vision fusion. Enable by setting LT_PORT_CFG to the connected serial port and
the matching SER_TELx_BAUD (module default: 921600).
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("linktrack", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS3", nullptr, "serial device", true);
	PRINT_MODULE_USAGE_PARAM_STRING('b', "921600", nullptr, "baud rate (or p:<param name>)", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int linktrack_main(int argc, char *argv[])
{
	return LinkTrack::main(argc, argv);
}
