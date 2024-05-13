import maya.cmds as cmds # type: ignore
import maya.api.OpenMaya as om # type: ignore


#Creates hierachy for controls and rig
def createHierarchy(exportJoints):
    #Top level groups for rig hierachy
    cmds.group(n="rig_grp", em=True)
    cmds.group(n="controls", em=True)
    cmds.group(n="rig_systems", em=True)
    cmds.parent("controls", "rig_systems", "rig_grp")

    #Controls group
    cmds.circle(n="root_ctrl", c=[0,0,0], nr=[0,1,0], r=50)
    cmds.parent("root_ctrl", "controls")

    #Connect root controller to root joint
    rel = cmds.listRelatives(exportJoints[0], f=True)
    firstRel = rel[0].split("|")
    print(firstRel[1])
    connectTransformAttr("root_ctrl", firstRel[1], [".scale", ".rotate", ".translate"])

    

    #Recolor root control group
    shapeNode = cmds.listRelatives("root_ctrl", shapes=True)[0]
    cmds.setAttr(f"{shapeNode}.overrideEnabled", 1)
    cmds.setAttr(f"{shapeNode}.overrideColor", 14)

    #Create groups for limbs and limb controls
    limbs = ["arm_l", "arm_r", "leg_l", "leg_r"]
    for limb in limbs:
        limbGroup = cmds.group(n=f"{limb}_grp", em=True)
        cmds.parent(limbGroup, "rig_systems")

        ctrlGroup = cmds.group(n=f"{limb}_ctrl_grp", em=True)
        cmds.parent(ctrlGroup, "root_ctrl")

#Returns a list of selected joints in the order they appear hierarchically. 
def sortJoints(selectedJoints):
    #Create a full list of relatives, sort to find the longest chain (which should contain all of our joints in the original selection). Split results from sort into list.
    relatives = cmds.listRelatives(selectedJoints,fullPath=True)
    relatives.sort(key=len, reverse=True)
    longestPath = relatives[0]
    relatives = longestPath.split("|")
    sortedJoints =  []
    firstJointIndex = 0
    #Iterate through hierarchy to find which joint from selectedJoints appears first in the chain, then break loop.
    for x in range(len(relatives)):
        for y in range(len(selectedJoints)):
            if str(relatives[x]) == str(selectedJoints[y]):
                firstJointIndex = x
                break
        else:
            continue
        break

    #Using index of the first instance of a selected joint in the hierarchy and the length of the selected joint list, create a new, sorted list.
    for x in range(len(selectedJoints)):
        sortedJoints.append(relatives[firstJointIndex + x])

    return sortedJoints

            




#Create fk joints
def createFkJoints(exportJoints, limbSide, limbType):
    if cmds.objExists(f"{exportJoints[0]}_fk"):
        raise Exception("FK Joints already exist, please delete existing joint and retry.")
    

    #Create fk joints and rename to match export joints 
    fkJoints = cmds.duplicate(exportJoints, po=True, renameChildren=True)
    if cmds.listRelatives(fkJoints[0], p=True):
        cmds.parent(fkJoints[0], w=True)

    tempJoints = []

    for joint in fkJoints:
        #Rename fk joints to match export joints
        matchJoint = exportJoints[fkJoints.index(joint)]
        suffix = "_fk"
        newName = cmds.rename(joint, matchJoint + suffix)
        tempJoints.append(newName)

    fkJoints = tempJoints

    parentLoc = limbType + "_" + limbSide[0] + "_grp"
    cmds.parent(fkJoints[0], parentLoc)

    return fkJoints

#Create fk controls and match to joint locations
def createFkControls(fkJoints, limbSide, limbType):
    fkControls = []

    for joint in fkJoints:
        #Create controls and match to corresponding joint locations
        newCircle = cmds.circle(n=f"{joint}_ctrl", c=[0,0,0], nr=[0,1,0], r=10)
        fkControls.append(f"{joint}_ctrl")
        cmds.matchTransform(newCircle, joint)
        
        #Parents circles 
        if fkJoints.index(joint) == 0:
            oldCircle = newCircle
        else:
            cmds.parent(newCircle, oldCircle)
            oldCircle = newCircle
    
    #Parent control to proper group
    cmds.parent(fkControls[0], f"{limbType}_{limbSide[0]}_ctrl_grp")
    
    #Recolor controls to corresponding side
    reColorControls(fkControls, limbSide)

    return fkControls

#Create ik joints
def createIkJoints(exportJoints, limbSide, limbType):
    if cmds.objExists(f"{exportJoints[0]}_ik"):
        raise Exception("IK Joints already exist, please delete existing joint and retry.")

    #Create fk joints and rename to match export joints 
    ikJoints = cmds.duplicate(exportJoints, po=True, renameChildren=True)
    if cmds.listRelatives(ikJoints[0], p=True):
        cmds.parent(ikJoints[0], w=True)

    tempJoints = []

    for joint in ikJoints:
        #Rename fk joints to match export joints
        matchJoint = exportJoints[ikJoints.index(joint)]
        suffix = "_ik"
        newName = cmds.rename(joint, matchJoint + suffix)
        tempJoints.append(newName)

    ikJoints = tempJoints

    #Move new IK joints into hierarchy
    parentLoc = limbType + "_" + limbSide[0] + "_grp"
    cmds.parent(ikJoints[0], parentLoc)

    #Clean transforms on ik joints
    for x in range(len(ikJoints)):
            clearTransforms(ikJoints[x])

    return ikJoints
    
#Creates ik controls and connect them to ik joints
def createIkControls(ikJoints, limbSide, limbType):
    #Establish which limb we're working on
    limb = f"{limbType}_{limbSide[0]}"

    #Create pole vector controller
    pv_ctrl_points = [(-5, 0, -5), (5, 0, -5), (5, 0, 5), (-5, 0, 5), (-5, 0, -5), (0, 10, 0),(5, 0, 5), (-5, 0, 5), (0, 10, 0), (5, 0, -5)]
    pv_curve = cmds.curve(n=f"{limb}_elbow_pv_ctrl",d=1, p=pv_ctrl_points, k=range(10))

    #Create and orient controls to specific to arm
    if limbType == "arm":
        #Create ik hand control curve
        ikCtrlPoints = [(-10, 0, -10), (0, 0, -7.5), (10, 0, -10), (7.5, 0, 0), (10, 0, 10), (0, 0, 7.5), (-10, 0, 10), (-7.5, 0, 0), (-10, 0, -10)]
        ikCtrlCurve = cmds.curve(n=f"{limb}_ik_ctrl",d=1, p=ikCtrlPoints, k=range(9))
        #ik control group
        ikControls = [f"{ikCtrlCurve}", f"{pv_curve}"]
        #Match ik control to wrist/foot joint. Align to floor if foot
        cmds.matchTransform(ikControls[0], ikJoints[-1])
    
    #Create and orient controls to specific to leg
    elif limbType == "leg":
        #Create ik foot control curve
        ikFootCtrlCurvePoints = [(0.84, 0, 14.9),(0.05, 0, 15.05),(-1.53, 0, 15.30),(-5.85, 0, 13.56),(-6.30, 0, 12.59),(-7.05, 0, 10.99),(-7.19, 0, 8.88),(-4.81, 0, 0.31),(-5.00, 0, -1.38),(-5.33, 0, -7.52),(-5.4, 0, -10.22),(-5.34, 0, -11.97),(-3.63, 0, -14.39), (-2.24, 0, -15.22),(0.76, 0, -15.49),(3.45, 0, -13.69), (5.27, 0, -10.87), (6.36, 0, -5.06),(6.46, 0, -2.13),(7.57, 0, 9.06), (5.13, 0, 13.49),(2.30, 0, 14.43), (0.89, 0, 14.91)]
        ikFootCtrlCurve = cmds.curve(n=f"{limb}_ik_ctrl", p=ikFootCtrlCurvePoints)
        #adjust orientation for right side
        if limbSide == "right":
            vertices = cmds.ls(cmds.ls(sl=True)[0] + '.cv[*]', flatten=True)
            cmds.select(vertices)
            cmds.rotate(-180, 0, -180, r=True, p=(0, 5, 0), os=True, fo=True)
            cmds.xform(f"{ikFootCtrlCurve}", cp=True)
            
        #ik control group
        ikControls = [f"{ikFootCtrlCurve}", f"{pv_curve}"]

        #Match ik control to wrist/foot joint. Align to floor if foot
        cmds.matchTransform(ikControls[0], ikJoints[-1])
        vertices = cmds.ls(ikControls[0] + '.cv[*]', flatten=True)
        cmds.select(vertices)
        distToFloor = cmds.xform(t=True, q=True, ws=True)[1] * -1
        moveForwardDist = 7
        if limbSide == "left":
            cmds.move(0, distToFloor, moveForwardDist, r=True, os=True, wd=0)
        elif limbSide == "right":
                cmds.move(0, distToFloor*-1, moveForwardDist*-1, r=True, os=True, wd=0)

    #Match PV controller to elbow/knee control
    if limbType == "arm":
        rotDistance = 90 if limbSide == "left" else -90
    else:
        rotDistance = -90 if limbSide == "left" else 90
    cmds.select(ikControls[1])
    vertices = cmds.ls(cmds.ls(sl=True)[0] + '.cv[*]', flatten=True)
    cmds.select(vertices)
    cmds.rotate(rotDistance, 0, 0, r=True, p=(0, 5, 0), os=True, fo=True)
    cmds.xform(ikControls[1], cp=True)
    cmds.matchTransform(ikControls[1], ikJoints[1])     

    #Create IK handle, parent to main ik control
    ikHandle = f"{limb}_ikHandle"
    cmds.ikHandle(n=ikHandle, sj=ikJoints[0], ee=ikJoints[-1])
    cmds.parent(ikHandle, ikControls[0])

    #Constrain IK handle to elbow/knee control, reposition pv control, orient wrist/foot to ikController
    cmds.poleVectorConstraint(ikControls[1], ikHandle)
    poleVectorAlign(ikControls[1], ikJoints[1], limbSide, limbType)
    cmds.orientConstraint(ikControls[0], ikJoints[-1])

    #Move ik controls into parent group
    cmds.parent(ikControls[0], ikControls[1], f"{limbType}_{limbSide[0]}_ctrl_grp")

    #Recolor controls to corresponding side
    reColorControls(ikControls, limbSide)

    #Clear transforms on IK controls
    for control in ikControls:
        clearTransforms(control)

    return ikControls

#Create a driver joint chain that is a duplicate of the ik joint chain for use in the quadruped ik rig.
def createDriverJoints(ikJoints):
    driverJoints = []

    for x in range(len(ikJoints)):
        jointName = f"{ikJoints[x]}_driver"
        driverJoints.append(jointName)

        tempJoint = cmds.duplicate(ikJoints[x], po=True)
        cmds.rename(tempJoint, driverJoints[x])

        if x != 0:
            cmds.parent(driverJoints[x], driverJoints[x-1])
    
    return driverJoints

#Creates an quadrupedal ik leg complete with secondary driver rig for ik limb that allows more fine tuned control of the extra joint using a hock control
def createQuadIkLeg(ikJoints, limbSide, limbType):
    limb = f"{limbType}_{limbSide[0]}"
    #Create secondary ik driver limb to drive quad limb
    driverJoints = createDriverJoints(ikJoints)

    #Create pole vector controller
    pv_ctrl_points = [(-5, 0, -5), (5, 0, -5), (5, 0, 5), (-5, 0, 5), (-5, 0, -5), (0, 10, 0),(5, 0, 5), (-5, 0, 5), (0, 10, 0), (5, 0, -5)]
    pv_curve = cmds.curve(n=f"{limb}_elbow_pv_ctrl",d=1, p=pv_ctrl_points, k=range(10))

    #Create ik foot control curve
    ikFootCtrlCurvePoints = [(0.84, 0, 14.9),(0.05, 0, 15.05),(-1.53, 0, 15.30),(-5.85, 0, 13.56),(-6.30, 0, 12.59),(-7.05, 0, 10.99),(-7.19, 0, 8.88),(-4.81, 0, 0.31),(-5.00, 0, -1.38),(-5.33, 0, -7.52),(-5.4, 0, -10.22),(-5.34, 0, -11.97),(-3.63, 0, -14.39), (-2.24, 0, -15.22),(0.76, 0, -15.49),(3.45, 0, -13.69), (5.27, 0, -10.87), (6.36, 0, -5.06),(6.46, 0, -2.13),(7.57, 0, 9.06), (5.13, 0, 13.49),(2.30, 0, 14.43), (0.89, 0, 14.91)]
    ikFootCtrlCurve = cmds.curve(n=f"{limb}_ik_ctrl", p=ikFootCtrlCurvePoints)
    #adjust orientation for right side
    if limbSide == "right":
        vertices = cmds.ls(cmds.ls(sl=True)[0] + '.cv[*]', flatten=True)
        cmds.select(vertices)
        cmds.rotate(-180, 0, -180, r=True, p=(0, 5, 0), os=True, fo=True)
        cmds.xform(f"{ikFootCtrlCurve}", cp=True)

    #Create hock control and orient to calf joint
    hockCtrl = cmds.circle(n=f"hock_{limbSide[0]}_ik_ctrl", c=[0,0,0], nr=[0,1,0], r=10)
    cmds.matchTransform(hockCtrl, ikJoints[2], pos=True)
        
    #ik control group
    ikControls = [f"{ikFootCtrlCurve}", f"{pv_curve}", f"{hockCtrl[0]}"]

    #Match ik control to wrist/foot joint. If foot joint is not placecd at y=0, this will move the CVs of the foot controller to the floor while keeping it's pivot at the location of the foot joint.
    cmds.matchTransform(ikControls[0], ikJoints[-1])
    vertices = cmds.ls(ikControls[0] + '.cv[*]', flatten=True)
    cmds.select(vertices)
    distToFloor = cmds.xform(t=True, q=True, ws=True)[1] * -1
    moveForwardDist = 7
    if limbSide == "left":
        cmds.move(0, distToFloor, moveForwardDist, r=True, os=True, wd=0)
    elif limbSide == "right":
            cmds.move(0, distToFloor*-1, moveForwardDist*-1, r=True, os=True, wd=0)

    #Parent hock control to foot controller. Clears transforms.
    cmds.parent(hockCtrl, f"{ikFootCtrlCurve}")
    clearTransforms(hockCtrl[0])

    #Match PV controller to elbow/knee control
    if limbType == "arm":
        rotDistance = 90 if limbSide == "left" else -90
    else:
        rotDistance = -90 if limbSide == "left" else 90
    cmds.select(ikControls[1])
    vertices = cmds.ls(cmds.ls(sl=True)[0] + '.cv[*]', flatten=True)
    cmds.select(vertices)
    cmds.rotate(rotDistance, 0, 0, r=True, p=(0, 5, 0), os=True, fo=True)
    cmds.xform(ikControls[1], cp=True)
    cmds.matchTransform(ikControls[1], ikJoints[1])

    #Create ik handle on driver limb joint chain.
    ikLegHandle = f"{limb}_ikHandle"
    cmds.ikHandle(n=ikLegHandle, sj=driverJoints[0], ee=driverJoints[-1])
    cmds.parent(ikLegHandle, ikControls[0])

    #Next, create ik handles for calf and hock that control the primary ik limb rig. We then parents these to the driver rig to allow for more fine control of the IK rig
    #Create calf ik handle and hock ik handle  
    ikCalfHandle = f"calf_{limbSide[0]}_ikHandle"
    cmds.ikHandle(n=ikCalfHandle, sj=ikJoints[0], ee=ikJoints[2])
    ikHockHandle = f"hock_{limbSide[0]}_ikHandle"
    cmds.ikHandle(n=ikHockHandle, sj=ikJoints[2], ee=ikJoints[3], sol="ikSCsolver")   
    #Parent ik handles to driver joints 
    cmds.parent(ikCalfHandle, driverJoints[2])
    cmds.parent(ikHockHandle, driverJoints[3])

    #Orient constrain foot joint to foot control
    cmds.orientConstraint(ikControls[0], ikJoints[-1])

    #Creates offset group for hock control and position at foot. Calf control will originate from the foot which will in turn lead to more stable movement on the leg when moving the hock controller 
    hockOffsetGroup = cmds.group(n=f"{ikControls[2]}_offset", em=True)
    cmds.matchTransform(hockOffsetGroup, hockCtrl)
    cmds.matchTransform(hockOffsetGroup, ikJoints[-1], pos=True)
    cmds.parent(hockOffsetGroup, driverJoints[2])
    clearTransforms(hockOffsetGroup)
    cmds.parent(ikCalfHandle, hockOffsetGroup)

    #Connect translate values from hock control to offset groups rotation values. When user pulls hock controls the offset group rotates from the base of the foot and moves the calf ik handle
    hockMult = f"hock_{limbSide[0]}_mult"
    cmds.createNode("multiplyDivide", n=hockMult)
    cmds.connectAttr(f"{ikControls[2]}.translate", f"{hockMult}.input1")
    cmds.connectAttr(f"{hockMult}.outputX", f"{hockOffsetGroup}.rotateZ")
    cmds.connectAttr(f"{hockMult}.outputZ", f"{hockOffsetGroup}.rotateX")
    cmds.setAttr(f"{hockMult}.input2X", -2)
    cmds.setAttr(f"{hockMult}.input2Z", 2)

    #Constrain IK handle to elbow/knee control, reposition pv control, orient wrist/foot to ikController
    cmds.poleVectorConstraint(ikControls[1], ikCalfHandle)
    poleVectorAlign(ikControls[1], ikJoints[1], limbSide, limbType)

    #Clean transforms on ik controls
    clearTransforms(ikControls[0])
    clearTransforms(ikControls[1])
    
    #Recolor controls to match limb side
    reColorControls(ikControls, limbSide)

    #Hide driver joint chain
    cmds.setAttr(f"{driverJoints[0]}.visibility", 0)


    #Move ik controls into parent group
    cmds.parent(ikControls[0], ikControls[1], f"{limbType}_{limbSide[0]}_ctrl_grp")

    return ikControls

#Connect each fkJoint in the chain to it's corresponding fk controller 
def connectFKControlsToJoints(fkJoints, fkControls):
    #Clear transforms on fk joints and corresponding controls
    for x in range(len(fkJoints)):
        clearTransforms(fkControls[x])
        clearTransforms(fkJoints[x])

    #Connects transform attributes on controls to joints
    for x in range(len(fkControls)):
        if x == 0:
            connectOffset(fkControls[x], fkJoints[x])
        else:
            connectTransformAttr(fkControls[x], fkJoints[x], [".translate", ".rotate"])    

#Creates a root controller at the "root" of each limb. Connect control to each rig system on each limb to allow simulatenous movement on each rig. Corrects any double transform issues inherited from root controller.
def createRootControl(exportJoints, ikJoints, fkJoints, limbSide, limbType):
    rootCtrl = "root_ctrl"
    rootExportJoint = exportJoints[0]
    limb = f"{limbType}_{limbSide[0]}"

    #Create control curve for root control
    if limbSide == "left":
        cmds.curve(n=f"{limb}_root_ctrl", d=1, p=((0,0,0), (-15, 0, 0), (-20, 5, 0), (-25, 0, 0), (-20, -5, 0), (-15, 0, 0)))
        rootLimbCtrl = f"{limb}_root_ctrl"
    elif limbSide == "right":
        cmds.curve(n=f"{limb}_root_ctrl", d=1, p=((0,0,0), (15, 0, 0), (20, 5, 0), (25, 0, 0), (20, -5, 0), (15, 0, 0)))
        rootLimbCtrl = f"{limb}_root_ctrl"

    #Recolor limb root control group
    shapeNode = cmds.listRelatives(rootLimbCtrl, shapes=True)[0]
    cmds.setAttr(f"{shapeNode}.overrideEnabled", 1)
    cmds.setAttr(f"{shapeNode}.overrideColor", 14)
    
    #Set controller scale, Move root control to match top of limb joint hierachy, move into controls group,and clean transforms
    controllerScale(f"{limb}_root_ctrl")
    cmds.matchTransform(rootLimbCtrl, rootExportJoint)
    clearTransforms(rootLimbCtrl)
    cmds.parent(rootLimbCtrl, f"{limb}_ctrl_grp")

    #Connect rootLimbControl as offset to ik/fk rig systems to allow for movement of whole rigged limb
    if fkJoints:
        #Multiply worldMatrix[0] attr from limb root ctrl and worldInverseMatrix[0] using mult matrix node
        fkMultMatrixNode = cmds.createNode("multMatrix", n=f"{fkJoints[0]}_multi")
        cmds.connectAttr(f"{rootLimbCtrl}.worldMatrix[0]", f"{fkMultMatrixNode}.matrixIn[0]")
        cmds.connectAttr(f"{rootCtrl}.worldInverseMatrix[0]", f"{fkMultMatrixNode}.matrixIn[1]")

        #Connect matrix sum from multMatrix node to offsetParentMatrix on fk controls to offset double transforms when moving root control
        cmds.connectAttr(f"{fkMultMatrixNode}.matrixSum", f"{fkJoints[0]}_ctrl.offsetParentMatrix")
    if ikJoints:
        #Connect worldMatrix[0] from root to ikJoint. No double transform correct necesssary on ik 
        cmds.connectAttr(f"{rootLimbCtrl}.worldMatrix[0]", f"{ikJoints[0]}.offsetParentMatrix")
        if len(exportJoints) == 4:
            cmds.connectAttr(f"{rootLimbCtrl}.worldMatrix[0]", f"{ikJoints[0]}_driver.offsetParentMatrix")

        

#Aligns pole vector elbow control behind arm at a position behind arm that keeps IK limb in it's initial position
def poleVectorAlign(pvControl, ikElbowJoint, limbSide, limbType):
    if limbType == "arm":
        moveDistance = -30 if limbSide == "left" else 30
    elif limbType == "leg":
        moveDistance = 30 if limbSide == "left" else -30
    #Create and align two temp locators to pv control, then move arbitrary distance behind the control
    cmds.spaceLocator(n="locator1")
    cmds.matchTransform("locator1", pvControl)
    cmds.move( 0, 0, moveDistance, "locator1", os=True, r=True)
    cmds.duplicate("locator1")

    #Parent locator to elbow joint, and align pv control to locator positon
    cmds.parent("locator2", ikElbowJoint)
    cmds.matchTransform(pvControl, "locator1")
    
    #Get current world pos for locators and pv control
    loc1 = cmds.xform("locator1", q=True, ws=True, translation=True)
    loc2 = cmds.xform("locator2", q=True, ws=True, translation=True)
    pvLoc = cmds.xform(pvControl, q=True, ws=True, translation=True)

    #Calculate distance betwwen locators & add to pole vector location
    locatorDistance = [loc1[0] - loc2[0], loc1[1] - loc2[1], loc1[2] - loc2[2]]
    newPosition = [pvLoc[0] + locatorDistance[0], pvLoc[1] + locatorDistance[1], pvLoc[2] + locatorDistance[2]]

    #Move pole vector control to new positon and delete temp locators
    cmds.xform(pvControl, translation=newPosition, worldSpace=True)
    cmds.delete("locator1")
    cmds.delete("locator2")


#Set offset parent matrix identity to transform values, then resets transform
def clearTransforms(node):
    #Create matrix from local transforms and offset parent matrix, reassign opm to new matrix
    localMatrix = om.MMatrix(cmds.xform(node, q=True, m=True, ws=False))
    offsetParentMatrix = om.MMatrix(cmds.getAttr(f"{node}.offsetParentMatrix"))
    bakedMatrix = localMatrix * offsetParentMatrix
    cmds.setAttr(f"{node}.offsetParentMatrix", bakedMatrix, type="matrix")
    transformList = [".translate", ".rotate", ".scale"]

    #Check if node has jointOrient attr that also needs to be cleared
    if cmds.attributeQuery('jointOrient', node=node, exists=True): transformList.append(".jointOrient")

    #Reset transform values
    for attribute in transformList:
        value = 1 if attribute == ".scale" else 0
        for axis in ["X", "Y", "Z"]:
            cmds.setAttr(node + attribute + axis, value)

#Connects worldMatrix attr from source to offsetParentMatrix of second
def connectOffset(sourceObject, targetObject):
    sourceAttr = f"{sourceObject}.worldMatrix[0]"
    targetAttr = f"{targetObject}.offsetParentMatrix"

    cmds.connectAttr(sourceAttr, targetAttr, force=True)

#Connects transforms attr specified in list from source to target
def connectTransformAttr(sourceObject, targetObject, attrList):
    for attr in attrList:
        sourceAttr = sourceObject + attr
        targetAttr = targetObject + attr
        
        cmds.connectAttr(sourceAttr, targetAttr, force=True)

def reColorControls(controlSet, limbSide):
    #Set color value to blue on left side and red on right
    colorVal = 18 if limbSide == "left" else 13

    for control in controlSet:
        shapeNode = cmds.listRelatives(control, shapes=True)[0]
        cmds.setAttr(f"{shapeNode}.overrideEnabled", 1)
        cmds.setAttr(f"{shapeNode}.overrideColor", colorVal)

#Constrain skeleton to created ik/fk rigs
def constrainSkeletonToRig(exportJoints, ikJoints, fkJoints):
    parentConstraints = []
    for x in range(len(exportJoints)):
        #Constrain both ik and fk joints
        if ikJoints and fkJoints:    
            newConstraint = cmds.parentConstraint(ikJoints[x], fkJoints[x], exportJoints[x], mo=False)
        else:
            #Constrain either just fk or just ik joints
            if ikJoints:
                newConstraint = cmds.parentConstraint(ikJoints[x], exportJoints[x], mo=False)
            if fkJoints:
                newConstraint = cmds.parentConstraint(fkJoints[x], exportJoints[x], mo=False)
        
        parentConstraints.append(newConstraint[0])
    
    return parentConstraints

#Create selectable ik/fk switch control with custom attributes
def createIKFKSwitcher(exportJoints, ikJoints, fkJoints, parentConstraints, limbSide, limbType):
    limb = f"{limbType}_{limbSide[0]}"

    #Set up ik/fk switcher control curves
    #Create curve for fk text
    fkTextCtrlPoints = [(0.0, 0, 0),(0, 12, 0),(6.66,12, 0),(6.66,10.5, 0),(1.6, 10.5, 0),(1.6, 6.6, 0),(5.8, 6.6, 0),(5.8, 5.5, 0),(1.5, 5.5, 0),(1.5, 0, 0),(0, 0, 0),(9, 0, 0),(9, 12, 0),(10.5, 12, 0),(10.5, 6, 0),(15, 12, 0),(17, 12, 0),(12, 6, 0),(17.8, 0, 0),(15.7, 0, 0),(10.5, 6, 0),(10.5, 0, 0),(9, 0, 0)]
    fkTextCurve = cmds.curve(n=f"{limb}_fk_txt", d=1, p=fkTextCtrlPoints)
    #Create curve for IK Text
    ikTextCtrlPoints = [(0, 0, 0),(0, 12, 0),(1.6, 12, 0),(1.6, 0, 0),(0, 0, 0),(4.8, 0, 0),(4.8, 12, 0),(6.4, 12, 0),(6.4, 6, 0),(11.2, 12, 0),(13, 12, 0),(8.25, 6.25, 0),(13.75, 0, 0),(11.6, 0, 0),(6.4, 6.0, 0),(6.4, 0, 0),(4.8, 0, 0)]
    ikTextCurve = cmds.curve(n=f"{limb}_ik_txt",d=1, p=ikTextCtrlPoints)
    #Create box to encapsulate text
    boxCtrlPoints = [[-2, -2, 0], [-2, 13, 0], [19, 13, 0], [19, -2, 0], [-2, -2, 0]]
    limbController = cmds.curve(n=f"{limb}_ctrl", d=1, p=boxCtrlPoints)
    #Parent text boxes to control box
    cmds.parent(f"{fkTextCurve}", f"{ikTextCurve}", f"{limbController}")
    
    #Set scale of controller
    controllerScale([f"{fkTextCurve}", f"{ikTextCurve}", f"{limbController}"])

    #Set switcher controller color
    reColorControls([f"{fkTextCurve}", f"{ikTextCurve}", f"{limbController}"], limbSide)

    #Make text not selectable on control not selectable
    cmds.setAttr(f"{fkTextCurve}.overrideEnabled", 1)
    cmds.setAttr(f"{fkTextCurve}.overrideDisplayType", 2)
    cmds.setAttr(f"{ikTextCurve}.overrideEnabled", 1)
    cmds.setAttr(f"{ikTextCurve}.overrideDisplayType", 2)


    #Add ik/fk switch attribute to limb controller
    cmds.addAttr(f"{limbController}", longName="IK_FK_Switch", attributeType="float", minValue=0.0, maxValue=1.0, defaultValue=0.0, keyable=True)

    #Create reverse node, connect ikfk switch attribute to input 1x on reverse node 
    ikfkReverseNode = cmds.createNode("reverse", n=f"{limb}_IKFK_Switch")
    cmds.connectAttr(f"{limbController}.IK_FK_Switch", f"{limb}_IKFK_Switch.inputX")

    #for each constraint in parentConstraints, connect ikFk switch attribute to ik weight attribute and the reversed output to the fk weight attribute
    for x in range(len(parentConstraints)):
        constraintIKWeight = f"{parentConstraints[x]}.{exportJoints[x]}_ikW0"
        constraintFKWeight = f"{parentConstraints[x]}.{exportJoints[x]}_fkW1"

        #Connect ikfk output to ik weight, and reverse output to fk weight
        cmds.connectAttr(f"{limbController}.IK_FK_Switch", constraintIKWeight)
        cmds.connectAttr(f"{ikfkReverseNode}.outputX", constraintFKWeight)
    
    #Connect controls visibility to ik fk switcher and reversed output    
    cmds.connectAttr(f"{limbController}.IK_FK_Switch", f"{limb}_ik_ctrl.visibility")
    cmds.connectAttr(f"{limbController}.IK_FK_Switch", f"{limb}_elbow_pv_ctrl.visibility")
    cmds.connectAttr(f"{ikfkReverseNode}.outputX", f"{fkJoints[0]}_ctrl.visibility")
    
    #Connect ik and fk joints visibility to ikfk switcher and reversed output
    cmds.connectAttr(f"{limbController}.IK_FK_Switch", f"{ikJoints[0]}.visibility")
    cmds.connectAttr(f"{ikfkReverseNode}.outputX", f"{fkJoints[0]}.visibility")
    
    #Connect ik and fk text visibility to switcher and reversed output
    cmds.connectAttr(f"{limbController}.IK_FK_Switch", f"{ikTextCurve}.visibility")
    cmds.connectAttr(f"{ikfkReverseNode}.outputX", f"{fkTextCurve}.visibility")
    
    #Move arm controller into place
    moveDistance = 10 if limbSide == "left" else -10
    cmds.matchTransform(f"{limbController}", exportJoints[-1], pos=True)
    cmds.move(moveDistance, 5, -25, f"{limbController}", r=True)
    cmds.scale(.625, .625, .625, f"{limbController}")
    cmds.makeIdentity(f"{limbController}", apply=True, t=0, r=0, s=1, n=0, pn=1)
    #Move into controls group and clear transforms
    cmds.parent(f"{limbController}", f"{limb}_ctrl_grp")
    clearTransforms(f"{limbController}")

    #Constrain switcher to follow the position of the wrist
    cmds.pointConstraint(exportJoints[-1], f"{limbController}", mo=True)
    
    

    return f"{limbController}"

#Set scale for control curve to controlScaleValue as determined by user input. Default value is 1.0. Scales CVs instead of objects to keep transforms cleans and not affect attached joints.
def controllerScale(controlSet):
    #Read input from UI for scale value
    controlScaleValue = float(cmds.floatField("controlScale", q=True, v=True))
    
    #For each control curve in the control set, iterate through each control vertex and set it's scale to controlScaleValue      
    if isinstance(controlSet, list):
        for x in range(len(controlSet)):
            cmds.select(controlSet[x])
            vertices = cmds.ls(cmds.ls(sl=True)[0] + '.cv[*]', flatten=True)
            #Iterate through vertices
            for y in range(len(vertices)):
                #On ik foot controls, only scale x and z axis
                if controlSet[x] == "leg_l_ik_ctrl" or controlSet[x] == "leg_r_ik_ctrl":
                    cmds.scale(controlScaleValue, 1, controlScaleValue,vertices[y])
                else:
                    cmds.scale(controlScaleValue, controlScaleValue,controlScaleValue,vertices[y])    
    #If not a list and only a single item
    else: 
        cmds.select(controlSet)
        vertices = cmds.ls(cmds.ls(sl=True)[0] + '.cv[*]', flatten=True)
        #Iterate through vertices
        for y in range(len(vertices)):
            cmds.scale(controlScaleValue, controlScaleValue,controlScaleValue,vertices[y])


#CREATE THE AUTO LIMBS
def autoLimbTool(*args):
    #Check Results of UI operation
    whichLimb = cmds.optionMenu("limbMenu", q=1, v=1).lower().split() 
    limbSide = whichLimb[0]
    limbType = whichLimb[1]
    
    #Check if creating ik or fk or both
    fkCheck = cmds.checkBox("fkCheck", q=1, v=1)
    ikCheck = cmds.checkBox("ikCheck", q=1, v=1)
    createIKFKSwitch = cmds.checkBox("ikfkSwitchCheck", q=1, v=1)

    #Check if user selection is less than three joints
    if len(cmds.ls(sl=True, type="joint")) < 3:
        raise Exception("\nPlease select at least three joints to continue")
    
    #Create export joints, organize list in hierarchical order    
    jointSelection=cmds.ls(sl=True, type="joint")
    exportJoints = sortJoints(jointSelection)
    

    #Creates group hierarchy on first pass of script
    if not cmds.objExists("rig_grp"):
        rootControl = createHierarchy(exportJoints)
        
    #Check if auto rigger has been ran on this limb by looking for parent constraint on export joints
    if cmds.objExists(f"{exportJoints[0]}_parentConstraint1"):
        raise Exception("Auto rigger has already been ran on this limb, please delete existing joints and constraints and retry.")

    #Create fk and IK limbs and controls
    if fkCheck:
        #Create fk controls and joints from export joints 
        fkJoints = createFkJoints(exportJoints, limbSide, limbType)
        fkControls = createFkControls(fkJoints, limbSide, limbType)
        connectFKControlsToJoints(fkJoints, fkControls) 
        #Set controller scale 
        controllerScale(fkControls)
    if ikCheck:
        ikJoints = createIkJoints(exportJoints, limbSide, limbType)
        if len(exportJoints) == 3:
            #Create a biped rig if three joints are selected
            ikControls = createIkControls(ikJoints, limbSide, limbType)
        elif len(exportJoints) == 4:
            #Create a quadruped rig if four joints are selected
            ikControls = createQuadIkLeg(ikJoints, limbSide, limbType)
        else: 
            raise Exception("Please select either three joints for biped rig or 4 joints for quadruped rig.")
        #Set ik controller scale
        controllerScale(ikControls)

    
    #Constrain created ik and fk rigs to selected joints, create root controls for each limb
    if fkCheck and ikCheck:
        parentConstraints = constrainSkeletonToRig(exportJoints, ikJoints, fkJoints)
        createRootControl(exportJoints, ikJoints, fkJoints, limbSide, limbType)
    elif ikCheck:
        parentConstraints = constrainSkeletonToRig(exportJoints, ikJoints, [])
        createRootControl(exportJoints, ikJoints, [], limbSide, limbType)
    elif fkCheck:
        parentConstraints = constrainSkeletonToRig(exportJoints, [], fkJoints)
        createRootControl(exportJoints, [], fkJoints, limbSide, limbType)

    #Create ik/fk switch control
    if createIKFKSwitch:
        if ikCheck and fkCheck:
            limbController = createIKFKSwitcher(exportJoints, ikJoints, fkJoints, parentConstraints, limbSide, limbType)
        else:
            raise Exception("Fk/IK Switch creator requires the creation of both ik and fk joints.")
        
    
        
    
    
    
    


#Create window UI for auto rig tool
def autoLimbRiggerToolUI():
    #Check if the window exists, and if it does, delete

    if(cmds.window("autoLimbRigToolUI", ex=1)):
        cmds.deleteUI("autoLimbRigToolUI")

    #Create window
    window = cmds.window("autoLimbRigToolUI", t="Auto Limb Rig Tool v1.0", w=200, h=200, mnb=0,mxb=0)

    #Create the main layout
    mainLayout = cmds.formLayout(nd=100)
   
    #Limb Menu
    limbMenu = cmds.optionMenu("limbMenu", l="Which Limb?", h=30, ann="Which limb are we working on?")

    cmds.menuItem("Left Arm")
    cmds.menuItem("Left Leg")
    cmds.menuItem("Right Arm")
    cmds.menuItem("Right Leg")

    #Checkboxes
    fkCheckBox = cmds.checkBox("fkCheck", l="Create FK limb?", h=20, ann="Create ik skeleton and controls?", v=0)
    ikCheckBox = cmds.checkBox("ikCheck", l="Create IK limb?", h=20, ann="Create fk skeleton and controls?", v=0)
    ikfkSwitchCheckBox = cmds.checkBox("ikfkSwitchCheck", l="Create ik/fk switch control?", h=20, ann="Create a controller that allows the user to switch between ik and fk.", v=0)
    
    scaleControl = cmds.floatField("controlScale", minValue=-10, maxValue=10, value= 1.0)
    scaleControlText = cmds.text("Control Scale")    

    #Separators
    separator01 = cmds.separator(h=5)
    separator02 = cmds.separator(h=5)
    separator03 = cmds.separator(h=5)
    separator04 = cmds.separator(h=5)

    #Buttons
    button = cmds.button(l="Apply", c=autoLimbTool)
    
    #Adjust layout
    cmds.formLayout(mainLayout, e=1, attachForm=[(limbMenu, 'top', 5), (limbMenu, 'left', 5), (limbMenu, 'right', 5),
                                                 (separator01, 'left', 5), (separator01, 'right', 5),
                                                 (separator02, 'left', 5), (separator02, 'right', 5),
                                                 (separator03, 'left', 5), (separator03, 'right', 5),
                                                 (separator04, 'left', 5), (separator04, 'right', 5),
                                                 (scaleControlText, 'left', 5), (scaleControl, 'right', 5),
                                                 (button, 'bottom', 5), (button, 'left', 5), (button, 'right', 5)
                                                 ],
                                                 
                                     attachControl=[(separator01, 'top', 5, limbMenu),
                                                    (fkCheckBox, 'top', 5, separator01),
                                                    (ikCheckBox, 'top', 5, separator01),
                                                    (separator02, 'top', 5, ikCheckBox),
                                                    (ikfkSwitchCheckBox , 'top', 5, separator02),
                                                    (separator03, 'top', 5, ikfkSwitchCheckBox ),
                                                    (scaleControl, 'top', 5, separator03),
                                                    (scaleControlText, 'top', 5, separator03),
                                                    (scaleControl, 'left', 5, scaleControlText),
                                                    (separator04, 'top', 5, scaleControl),
                                                    (button, 'top', 5, separator04),
                                                   ],

                                     attachPosition=[(fkCheckBox, 'left', 5, 10),
                                                     (ikCheckBox, 'right', 5, 90),
                                                     (ikfkSwitchCheckBox, 'left', 5, 25)
                                                    ]           
                                                 
                                                 )

    #Display window
    cmds.showWindow(window)
