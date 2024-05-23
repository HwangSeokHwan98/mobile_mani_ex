from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D

my_chain = Chain.from_urdf_file("/home/hsh/catkin_ws/src/ur_ikfast/ur3/ur3.urdf")

desired_joint=my_chain.inverse_kinematics([0.00575118, 0.1688, 0.1688])

joint_positions = [
        desired_joint[0],
        desired_joint[1],
        desired_joint[2],
        desired_joint[3],
        desired_joint[4],
        desired_joint[5]
    ]

print(joint_positions)
print(desired_joint)

ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')
my_chain.plot(my_chain.inverse_kinematics([0.00575118, 1.1688, 1.1688]), ax)
matplotlib.pyplot.show()