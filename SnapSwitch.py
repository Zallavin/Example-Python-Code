import math

from maya import OpenMaya as om
from pymel.core import 

try
    from PySide2.QtCore import 
    from PySide2.QtGui import 
    from PySide2.QtWidgets import 
    from PySide2.QtUiTools import 
    from shiboken2 import wrapInstance
except ImportError
    from PySide.QtCore import 
    from PySide.QtGui import 
    from PySide.QtUiTools import 
    from shiboken import wrapInstance

from maya.app.general.mayaMixin import MayaQWidgetDockableMixin
from functools import partial


def match_fk_to_bound(side=None, bodypart=None, args)
    iter_list = []

    arm_components = [('BN_{}_UpperArm_01', 'FK_{}_UpperArm_CTRL_01'), ('BN_{}_ForeArm_01', 'FK_{}_ForeArm_CTRL_01'),
                      ('BN_{}_Wrist_01', 'FK_{}_Wrist_CTRL_01')]

    leg_components = [('BN_{}_Thigh_01', 'FK_{}_Thigh_CTRL_01'), ('BN_{}_Shin_01', 'FK_{}_Shin_CTRL_01'),
                      ('BN_{}_Ankle_01', 'FK_{}_Ankle_CTRL_01'),
                      ('BN_{}_InnerToe_A_01', 'FK_{}_InnerToe_A_CTRL_01'),
                      ('BN_{}_InnerToe_B_01', 'FK_{}_InnerToe_B_CTRL_01'),
                      ('BN_{}_OuterToe_A_01', 'FK_{}_OuterToe_A_CTRL_01'),
                      ('BN_{}_OuterToe_B_01', 'FK_{}_OuterToe_B_CTRL_01')]

    if bodypart == 'Arm'
        iter_list = arm_components

    else
        iter_list = leg_components

    # Align the rotation of the control to the one of the corresponding joint
    for ref_obj, ctrl in iter_list
        rotation = PyNode(ref_obj.format(side)).getRotation(space='world')
        PyNode(ctrl.format(side)).setRotation(rotation, space='world')

    PyNode('{}_{}_Settings_CTRL_01.IK_FK_Blend'.format(bodypart, side)).set(1)


def match_ik_to_bound(side=None, bodypart=None, args)
    component_list = []
    jnt_list = []

    arm_components = [('IK_{}_Wrist_CTRL_Snap_CRV_01', 'IK_{}_Wrist_CTRL_01')]

    leg_components = [('IK_{}_Foot_CTRL_Snap_CRV_01', 'IK_{}_Foot_CTRL_01'),
                      ('IK_{}_InnerToe_CTRL_Snap_CRV_01', 'IK_{}_InnerToe_CTRL_01'),
                      ('IK_{}_OuterToe_CTRL_Snap_CRV_01', 'IK_{}_OuterToe_CTRL_01')]

    arm_jnts = ['BN_{}_UpperArm_01', 'BN_{}_ForeArm_01', 'BN_{}_Wrist_01']
    leg_jnts = ['BN_{}_Thigh_01', 'BN_{}_Shin_01', 'BN_{}_Ankle_01']

    if bodypart == 'Arm'
        component_list = arm_components
        jnt_list = arm_jnts

    else
        component_list = leg_components
        jnt_list = leg_jnts

    # Align the IK-controls to the corresponding reference objects
    for ref_obj, ctrl in component_list
        translation = PyNode(ref_obj.format(side)).getTranslation(space='world')
        PyNode(ctrl.format(side)).setTranslation(translation, space='world')

        rotation = PyNode(ref_obj.format(side)).getRotation(space='world')
        PyNode(ctrl.format(side)).setRotation(rotation, space='world')

    # Aligning the pole vectors

    # Getting world translation values of bound joints
    start_v = PyNode(jnt_list[0].format(side)).getTranslation(space='world')
    mid_v = PyNode(jnt_list[1].format(side)).getTranslation(space='world')
    end_v = PyNode(jnt_list[2].format(side)).getTranslation(space='world')

    # Getting relative vectors between the joints
    start_end_v = end_v - start_v
    start_mid_v = mid_v - start_v

    # Get the arrow vector via vector dot product and the projection
    dot_prod = start_mid_v  start_end_v

    proj = float(dot_prod)  float(start_end_v.length())
    start_end_norm_v = start_end_v.normal()
    proj_v = start_end_norm_v  proj

    arrow_v = start_mid_v - proj_v

    # Normalizing vector for the matrix values of one the orientation axes
    arrow_v.normalize()

    # Get UI value for the pole vector distance
    pv_distance = floatField('pv_distance', query=True, value=True)
    pole_distance_v = arrow_v  pv_distance

    # Position of the pole vector control
    final_v = pole_distance_v + mid_v

    # Setting the translation values
    PyNode('IK_{}_{}_PoleV_CTRL_01'.format(side, bodypart)).setTranslation(final_v, space='world')

    if checkBox('pv_orient', query=True, value=True)
        # Getting vectors for the orientation axes to compose a matrix
        cross_prod_x = start_end_v ^ start_mid_v
        cross_prod_x.normalize()

        cross_prod_z = cross_prod_x ^ arrow_v
        cross_prod_z.normalize()

        if bodypart == 'Arm'
            arrow_v_y = arrow_v  -1

        elif bodypart == 'Leg'
            arrow_v_y = arrow_v

        # Composing the matrix
        matrix_v = [cross_prod_x.x, cross_prod_x.y, cross_prod_x.z, 0,
                    arrow_v_y.x, arrow_v_y.y, arrow_v_y.z, 0,
                    cross_prod_z.x, cross_prod_z.y, cross_prod_z.z, 0,
                    0, 0, 0, 1]

        matrix_m = om.MMatrix()
        om.MScriptUtil.createMatrixFromList(matrix_v, matrix_m)
        matrix_fn = om.MTransformationMatrix(matrix_m)

        rot = matrix_fn.eulerRotation()

        # Setting the rotation values
        PyNode('IK_{}_{}_PoleV_CTRL_01'.format(side, bodypart)).setRotation(rot.asVector()  math.pi  180.0,
                                                                            space='world')

    PyNode('{}_{}_Settings_CTRL_01.IK_FK_Blend'.format(bodypart, side)).set(0)


class SnapSwitchUI(MayaQWidgetDockableMixin, QWidget)
    def __init__(self, parent=None)
        super(SnapSwitchUI, self).__init__(parent=parent)

        self.setObjectName('Snap_Switch_UI')
        self.setWindowTitle('SnapSwitch v. 1.0')
        self.setContentsMargins(0, 0, 0, 0)

    def run(self)
        workspaceControlName = self.objectName() + 'WorkspaceControl'
        self.deleteControl(workspaceControlName)

        self.create_ui()

    @staticmethod
    def create_ui()
        template = uiTemplate('snapSwitchTemplate', force=True)
        template.define(button, width=80, height=30)
        template.define(frameLayout, collapsable=True, marginHeight=0, marginWidth=0)

        setParent('Snap_Switch_UI')

        columnLayout(adjustableColumn=True, columnOffset=('both', 1))

        with template
            with frameLayout(label='Arm')
                with formLayout() as arm_layout
                    b_1 = button(label='Left FK arm', command=partial(match_fk_to_bound, 'L', 'Arm'))
                    b_2 = button(label='Right FK arm', command=partial(match_fk_to_bound, 'R', 'Arm'))
                    b_3 = button(label='Left IK arm', command=partial(match_ik_to_bound, 'L', 'Arm'))
                    b_4 = button(label='Right IK arm', command=partial(match_ik_to_bound, 'R', 'Arm'))

                    arm_layout.attachForm(b_1, 'top', 1)
                    arm_layout.attachForm(b_1, 'left', 0)
                    arm_layout.attachPosition(b_1, 'right', 0, 50)
                    arm_layout.attachPosition(b_1, 'bottom', 0, 50)

                    arm_layout.attachForm(b_2, 'top', 1)
                    arm_layout.attachForm(b_2, 'right', 0)
                    arm_layout.attachControl(b_2, 'left', 2, b_1)
                    arm_layout.attachPosition(b_2, 'bottom', 0, 50)

                    arm_layout.attachControl(b_3, 'top', 2, b_1)
                    arm_layout.attachForm(b_3, 'left', 0)
                    arm_layout.attachPosition(b_3, 'right', 0, 50)
                    arm_layout.attachForm(b_3, 'bottom', 2)

                    arm_layout.attachControl(b_4, 'top', 2, b_2)
                    arm_layout.attachForm(b_4, 'right', 0)
                    arm_layout.attachControl(b_4, 'left', 2, b_3)
                    arm_layout.attachForm(b_4, 'bottom', 2)

            with frameLayout(label='Leg')
                with formLayout() as leg_layout
                    b_1 = button(label='Left FK leg', command=partial(match_fk_to_bound, 'L', 'Leg'))
                    b_2 = button(label='Right FK leg', command=partial(match_fk_to_bound, 'R', 'Leg'))
                    b_3 = button(label='Left IK leg', command=partial(match_ik_to_bound, 'L', 'Leg'))
                    b_4 = button(label='Right IK leg', command=partial(match_ik_to_bound, 'R', 'Leg'))

                    leg_layout.attachForm(b_1, 'top', 1)
                    leg_layout.attachForm(b_1, 'left', 0)
                    leg_layout.attachPosition(b_1, 'right', 0, 50)
                    leg_layout.attachPosition(b_1, 'bottom', 0, 50)

                    leg_layout.attachForm(b_2, 'top', 1)
                    leg_layout.attachForm(b_2, 'right', 0)
                    leg_layout.attachControl(b_2, 'left', 2, b_1)
                    leg_layout.attachPosition(b_2, 'bottom', 0, 50)

                    leg_layout.attachControl(b_3, 'top', 2, b_1)
                    leg_layout.attachForm(b_3, 'left', 0)
                    leg_layout.attachPosition(b_3, 'right', 0, 50)
                    leg_layout.attachForm(b_3, 'bottom', 2)

                    leg_layout.attachControl(b_4, 'top', 2, b_2)
                    leg_layout.attachForm(b_4, 'right', 0)
                    leg_layout.attachControl(b_4, 'left', 2, b_3)
                    leg_layout.attachForm(b_4, 'bottom', 2)

            with frameLayout(label='Options', collapse=True, marginHeight=1, marginWidth=0)
                with columnLayout(adjustableColumn=True, columnOffset=('both', 1), backgroundColor=[0.35, 0.35, 0.35])
                    with formLayout() as options_layout
                        t_1 = text(label='PV Distance', align='left')
                        ff_1 = floatField('pv_distance', minValue=0, maxValue=5, value=4, step=.1, precision=1,
                                          backgroundColor=[0.17, 0.17, 0.17])
                        t_2 = text(label='PV Orient', align='left')
                        cb_1 = checkBox('pv_orient', label='', align='center')

                        options_layout.attachForm(t_1, 'top', 1)
                        options_layout.attachForm(t_1, 'left', 0)
                        options_layout.attachPosition(t_1, 'right', 0, 50)
                        options_layout.attachPosition(t_1, 'bottom', 0, 50)

                        options_layout.attachForm(ff_1, 'top', 1)
                        options_layout.attachForm(ff_1, 'right', 0)
                        options_layout.attachControl(ff_1, 'left', 2, t_1)
                        options_layout.attachPosition(ff_1, 'bottom', 0, 50)

                        options_layout.attachControl(t_2, 'top', 2, t_1)
                        options_layout.attachForm(t_2, 'left', 0)
                        options_layout.attachPosition(t_2, 'right', 0, 50)
                        options_layout.attachForm(t_2, 'bottom', 2)

                        options_layout.attachControl(cb_1, 'top', 2, ff_1)
                        options_layout.attachForm(cb_1, 'right', 0)
                        options_layout.attachControl(cb_1, 'left', 2, t_2)
                        options_layout.attachForm(cb_1, 'bottom', 2)

    def deleteControl(self, control)
        if workspaceControl(control, q=True, exists=True)
            workspaceControl(control, e=True, close=True)
            deleteUI(control, control=True)


def main()
    ui = SnapSwitchUI()
    ui.run()
    ui.show(dockable=True)


if __name__ == '__main__'
    main()