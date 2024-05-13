# ikfk-limb-rigger

Created by Louis Whitworth, 2024.

I am an aspiring technical animator/rigging artist and I am currently looking for entry-level work. If you're interested in hiring me, please shoot me an email at whitworthlouis@gmail.com and I'd be happy to share my resume with you. 

-------------------------------------------------------------------------------------------------------------------------------------------------------------

This is version 1.0 of my auto-rigging tool in Maya. As of right now it only works on arms and legs, but I intend on continuing to build it out over time until it is capable of rigging
and an entire biped rig. 

As of now, it can:
- Creates fully functional arm and leg rigs with IK/FK blending
- Can rig both Bipedal and Quadrupedal legs
- Creates custom control curves that can be sized up or down to fit the model/scene scale
- Manages transform offsets with the Offset Parent Matrix
- Organizes all of the joints and controls into a hierarchy

Important Notes:
This tool does not yet have the capability of self-adjusting to any kind of joint orientation. So for the time being it must be used with the following joint orientation:
- Primary Axis: Y
- Secondary Axis: Z
- (Meaning Y is the axis that points down the joints, Z points forward from the joint, and X points down.)

A demo of this tool in action can be seen on my Artstation page: https://www.artstation.com/artwork/aoWEvL



