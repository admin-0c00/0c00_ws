#!/usr/bin/env python3
"""解析雀.stp 装配体：输出零件实例树（名称/位置/包围盒）。"""
import sys

from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDF import TDF_Label, TDF_LabelSequence
from OCP.TDataStd import TDataStd_Name
from OCP.TDocStd import TDocStd_Document
from OCP.TopLoc import TopLoc_Location
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ShapeTool


def get_name(label):
    attr = TDataStd_Name()
    if label.FindAttribute(TDataStd_Name.GetID_s(), attr):
        return attr.Get().ToExtString()
    return "?"


def bbox_of(shape):
    box = Bnd_Box()
    BRepBndLib.Add_s(shape, box)
    xmin, ymin, zmin, xmax, ymax, zmax = box.Get()
    return (xmin, ymin, zmin, xmax, ymax, zmax)


def walk(shape_tool, label, parent_loc, depth, out):
    loc = parent_loc.Multiplied(XCAFDoc_ShapeTool.GetLocation_s(label))
    name = get_name(label)
    if shape_tool.IsAssembly_s(label):
        comps = TDF_LabelSequence()
        shape_tool.GetComponents_s(label, comps)
        for i in range(1, comps.Length() + 1):
            walk(shape_tool, comps.Value(i), loc, depth + 1, out)
    else:
        ref = TDF_Label()
        if shape_tool.GetReferredShape_s(label, ref):
            prod_name = get_name(ref)
        else:
            prod_name = name
        shape = shape_tool.GetShape_s(label)
        shape = shape.Moved(loc)
        trsf = loc.Transformation()
        t = trsf.TranslationPart()
        bb = bbox_of(shape)
        out.append((depth, prod_name, (t.X(), t.Y(), t.Z()), bb, shape, loc))


def main(path):
    reader = STEPCAFControl_Reader()
    assert reader.ReadFile(path) == 1, "STEP 读取失败"
    doc = TDocStd_Document(TCollection_ExtendedString("doc"))
    assert reader.Transfer(doc), "STEP 转换失败"
    shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    roots = TDF_LabelSequence()
    shape_tool.GetFreeShapes(roots)
    out = []
    for i in range(1, roots.Length() + 1):
        walk(shape_tool, roots.Value(i), TopLoc_Location(), 0, out)
    print(f"{'深度':<3}{'零件':<16}{'实例位置 x,y,z (mm)':<28}包围盒尺寸 (mm)")
    for depth, name, t, bb, _, _ in out:
        size = (bb[3] - bb[0], bb[4] - bb[1], bb[5] - bb[2])
        print(f"{depth:<4}{name:<16}({t[0]:7.1f},{t[1]:7.1f},{t[2]:7.1f})    "
              f"{size[0]:.1f} x {size[1]:.1f} x {size[2]:.1f}")
    return out


if __name__ == "__main__":
    main(sys.argv[1])
