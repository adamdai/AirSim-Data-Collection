# In settings.json first activate computer vision mode: 
# https://github.com/Microsoft/AirSim/blob/master/docs/image_apis.md#computer-vision-mode

import setup_path 
import airsim

import pprint
import numpy as np

from utility import airsimVec2npArr

client = airsim.VehicleClient()
client.confirmConnection()

# objects can be named in two ways:
# 1. In UE Editor, select and object and change its name to something else. Note that you must *change* its name because
#    default name is auto-generated and varies from run-to-run.
# 2. OR you can do this: In UE Editor select the object and then go to "Actor" section, click down arrow to see "Tags" property and add a tag there.
#
# The simGetObjectPose and simSetObjectPose uses first object that has specified name OR tag.
# more info: https://answers.unrealengine.com/questions/543807/whats-the-difference-between-tag-and-tag.html
#            https://answers.unrealengine.com/revisions/790629.html

# below works in Blocks environment


range = 10

objs = client.simListSceneObjects()

for obj in objs:
    pose = client.simGetObjectPose(obj)
    dist = np.linalg.norm(airsimVec2npArr(pose.position))
    if dist < range:
        print("%s - Position: %s, Orientation: %s" % (pprint.pformat(obj), pprint.pformat(pose.position),
            pprint.pformat(pose.orientation)))

