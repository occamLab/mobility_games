#!/usr/bin/env python
from dynamic_reconfigure.parameter_generator_catkin import *
import os
from rospkg import RosPack

def soundFinder(gen):
    top = RosPack().get_path("mobility_games")
    sounds = [];
    firstsound = None;
    for dirname, dirnames, filenames in os.walk(top + '/auditory/sound_files'):
        first = True
        for fname in filenames:
            if fname.endswith('.wav'):
                if first:
                    firstsound = os.path.join(dirname, fname)
                    first = False
                sounds.append(gen.const(fname[:-4], str_t, os.path.join(dirname, fname), fname))
                print(fname[:-4])
    sound_enum = gen.enum(sounds, "An enum to set .wav Sound")
    gen.add("rewardSound", str_t, 0, "Filename of Sound file to be used as reward", firstsound, edit_method=sound_enum)
