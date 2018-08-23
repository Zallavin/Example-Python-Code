from maya import cmds, OpenMaya

# This script creates the required nodes for the lips and eyebrows
# Uses pointOnCurveInfo nodes to drive the joint translations and matrix nodes to drive the rotations

# Selection list of the relevant joints
sel = cmds.ls(sl=1)

# Curve that drives the translation of the joints (Param-range 0 to 1)
crv = "Right_Eyebrow_CRV_0Shape1"
# Curve that helps to stabilize the joint rotations (Param-range 0 to 1)
oCrv = "Right_Eyebrow_orientCRV_0Shape1"


def getUParam(pnt=[], crv=None):

    point = OpenMaya.MPoint(pnt[0], pnt[1], pnt[2])

    # Initialize function set to the MDagPath-object of the curve
    curveFn = OpenMaya.MFnNurbsCurve(getDagPath(crv))
    paramUtil = OpenMaya.MScriptUtil()
    paramPtr = paramUtil.asDoublePtr()
    isOnCurve = curveFn.isPointOnCurve(point)
    if isOnCurve:

        curveFn.getParamAtPoint(point, paramPtr, 0.001, OpenMaya.MSpace.kObject)
    else:
        point = curveFn.closestPoint(point, paramPtr, 0.001, OpenMaya.MSpace.kObject)
        curveFn.getParamAtPoint(point, paramPtr, 0.001, OpenMaya.MSpace.kObject)
    
    param = paramUtil.getDouble(paramPtr)
    return param


def getDagPath(objectName):
    
    if isinstance(objectName, list) == True:
        oNodeList = []
        for o in objectName:
            selectionList = OpenMaya.MSelectionList()
            selectionList.add(o)
            oNode = OpenMaya.MDagPath()
            selectionList.getDagPath(0, oNode)
            oNodeList.append(oNode)
        return oNodeList
    else:
        selectionList = OpenMaya.MSelectionList()
        selectionList.add(objectName)
        oNode = OpenMaya.MDagPath()
        selectionList.getDagPath(0, oNode)
        return oNode


for s in sel:
    pos = cmds.xform(s, q=1, ws=1, t=1)
    uParam = getUParam(pos, crv)

    # Creating required Nodes

    # The pointOnCurveInfo-node returns the world space translation at a given parameter
    # and will drive the joint translation
    pci = cmds.createNode("pointOnCurveInfo", n=s.replace("_JNT", "_PCI"))
    oPci = cmds.createNode("pointOnCurveInfo", n=s.replace("_JNT", "_orientPCI"))
    # The matrix will be fed with vectors to orient the joints according to the curves
    mtx = cmds.createNode("fourByFourMatrix", n=s.replace("_JNT", "_ROT_MTX"))
    decMtx = cmds.createNode("decomposeMatrix", n=s.replace("_JNT", "_DEC_ROT_MTX"))

    pma = cmds.createNode("plusMinusAverage", n=s.replace("_JNT", "_orientSUB"))

    # Assign the same parameter to both pointOnCurveInfo-nodes
    cmds.setAttr(pci + '.parameter', uParam)
    cmds.setAttr(oPci + '.parameter', uParam)
    # Set operation to subtract
    cmds.setAttr(pma + '.operation', 2)

    cmds.connectAttr(crv + '.worldSpace', pci + '.inputCurve')
    cmds.connectAttr(oCrv + '.worldSpace', oPci + '.inputCurve')
    cmds.connectAttr(oPci + '.position', pma + '.input3D[0]')
    cmds.connectAttr(pci + '.position', pma + '.input3D[1]')
    # Feed joint with translation values
    cmds.connectAttr(pci + '.position', s + '.t')

    # Compose matrix with the required values for stable rotations
    # Tangent vector of the curve at the given parameter (x-axis)
    cmds.connectAttr(pci + '.normalizedTangentX', mtx + '.in00')
    cmds.connectAttr(pci + '.normalizedTangentY', mtx + '.in01')
    cmds.connectAttr(pci + '.normalizedTangentZ', mtx + '.in02')
    # Vector that points from the normal curve to the orient curve at the given parameter(y-axis)
    cmds.connectAttr(pma + '.output3Dx', mtx + '.in20')
    cmds.connectAttr(pma + '.output3Dy', mtx + '.in21')
    cmds.connectAttr(pma + '.output3Dz', mtx + '.in22')

    cmds.connectAttr(mtx + '.output', decMtx + '.inputMatrix')
    # Feed joint with rotation values
    cmds.connectAttr(decMtx + '.outputRotate', s + '.r')
