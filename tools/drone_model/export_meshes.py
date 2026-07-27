#!/usr/bin/env python3
"""把雀.stp 装配体和 DT90 桨叶导出为 STL（视觉/碰撞）+ 生成三视图投影图。

输出（drone_model/out/）:
  visual_*.stl   各零件视觉网格（0.3mm 精度）
  collision.stl  全机碰撞网格（1.2mm 精度，合并壳体）
  prop_cw.stl / prop_ccw.stl  桨叶网格
  view_top.png / view_front.png / view_side.png  投影图（判机头方向用）
"""
import os
import sys

from OCP.Bnd import Bnd_Box
from OCP.BRep import BRep_Builder
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.StlAPI import StlAPI_Writer
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDF import TDF_Label, TDF_LabelSequence
from OCP.TDataStd import TDataStd_Name
from OCP.TDocStd import TDocStd_Document
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS_Compound
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ShapeTool

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")
os.makedirs(OUT, exist_ok=True)


def get_name(label):
    attr = TDataStd_Name()
    if label.FindAttribute(TDataStd_Name.GetID_s(), attr):
        return attr.Get().ToExtString()
    return "?"


def walk(shape_tool, label, parent_loc, out):
    loc = parent_loc.Multiplied(XCAFDoc_ShapeTool.GetLocation_s(label))
    if shape_tool.IsAssembly_s(label):
        comps = TDF_LabelSequence()
        shape_tool.GetComponents_s(label, comps)
        for i in range(1, comps.Length() + 1):
            walk(shape_tool, comps.Value(i), loc, out)
    else:
        ref = TDF_Label()
        if shape_tool.GetReferredShape_s(label, ref):
            name = get_name(ref)
            # 关键：取“被引用产品”的未定位形状，再由我们沿装配链累积的 loc 定位；
            # 若直接 GetShape_s(组件标签) 会自带一层定位，导致嵌套装配重复变换
            shape = shape_tool.GetShape_s(ref).Moved(loc)
        else:
            name = get_name(label)
            shape = shape_tool.GetShape_s(label).Moved(loc)
        out.append((name, shape))


def load_step(path):
    reader = STEPCAFControl_Reader()
    assert reader.ReadFile(path) == 1
    doc = TDocStd_Document(TCollection_ExtendedString("doc"))
    assert reader.Transfer(doc)
    shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    roots = TDF_LabelSequence()
    shape_tool.GetFreeShapes(roots)
    out = []
    for i in range(1, roots.Length() + 1):
        walk(shape_tool, roots.Value(i), TopLoc_Location(), out)
    return out


def export_stl(shape, path, deflection):
    BRepMesh_IncrementalMesh(shape, deflection, False, 0.4, True)
    assert StlAPI_Writer().Write(shape, path), f"STL 写出失败: {path}"


def volume_of(shape):
    from OCP.GProp import GProp_GProps
    from OCP.BRepGProp import BRepGProp
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, props)
    return props.Mass()  # mm^3


def main():
    parts = load_step(os.path.join(os.path.dirname(os.path.abspath(__file__)), "雀.stp"))
    builder = BRep_Builder()
    compound = TopoDS_Compound()
    builder.MakeCompound(compound)
    print(f"{'零件':<14}{'体积(cm³)':>10}")
    total_vol = 0.0
    safe_idx = {}
    for name, shape in parts:
        vol = volume_of(shape) / 1000.0
        total_vol += vol
        print(f"{name:<16}{vol:>9.1f}")
        builder.Add(compound, shape)
        # 视觉网格按零件导出（名字转安全文件名）
        safe_idx[name] = safe_idx.get(name, 0) + 1
        safe = f"{safe_idx[name]:02d}_{name}"
        export_stl(shape, os.path.join(OUT, f"visual_{safe}.stl"), 0.3)
    print(f"{'合计':<16}{total_vol:>9.1f}")
    export_stl(compound, os.path.join(OUT, "collision.stl"), 1.2)
    export_stl(compound, os.path.join(OUT, "visual_all.stl"), 0.3)

    here = os.path.dirname(os.path.abspath(__file__))
    for src, dst in (("DT90.stp", "prop_cw.stl"), ("DT90_ccw.stp", "prop_ccw.stl")):
        for name, shape in load_step(os.path.join(here, src)):
            export_stl(shape, os.path.join(OUT, dst), 0.2)
    print("STL 导出完成 ->", OUT)


if __name__ == "__main__":
    main()
