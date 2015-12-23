from hpp.corbaserver.manipulation.ur5 import Robot
from hpp.corbaserver.manipulation import ProblemSolver, ConstraintGraph
from hpp.gepetto.manipulation import Viewer, ViewerFactory
from hpp.gepetto import PathPlayer

# Load PR2 and a box to be manipulated. {{{2
class Box (object):
  rootJointType = 'freeflyer'
  packageName = 'hpp_tutorial'
  meshPackageName = 'hpp_tutorial'
  urdfName = 'box'
  urdfSuffix = ""
  srdfSuffix = ""

class Environment (object):
  packageName = 'fiad_description'
  meshPackageName = 'fiad_description'
  urdfName = 'tampon_cell'
  urdfSuffix = ""
  srdfSuffix = ""

robot = Robot ('ur5-box', 'ur5')
robot.client.manipulation.robot.setRootJointPosition \
    ('ur5', (0.115, 0.78, 0, 1, 0, 0, 0))

ps = ProblemSolver (robot)
vf = ViewerFactory (ps)

vf.loadObjectModel (Box, 'box')
vf.loadEnvironmentModel (Environment, "tampon_cell")

table_size_x=0.78
table_size_y=1.56
table_size_z=0.755

vf.moveObstacle ('tampon_cell/base_link_0', (table_size_x/2, table_size_y/2,
                                             -table_size_z/2, 1, 0, 0, 0))

robot.removeObstacleFromJoint ('tampon_cell/base_link_0',
                               'ur5/shoulder_pan_joint')
robot.removeObstacleFromJoint ('tampon_cell/base_link_0', 'ur5/world_joint')

viewer = vf.createViewer ()

q_init = robot.getCurrentConfig ()
q_init [6:9] = [0.6, 0.4, 0.03]

viewer (q_init)
