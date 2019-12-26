## CAPTURE AUDIOS<br>
* This node is intended to be used to create the dataset of the `rz_se` package.
* Firstly, audios must be captured using the robot microphone. The folder structure for the training set should have as many subfolders as emotions need to be detected, so that each subfolder will store the audios of the same emotion. This rule has to be applied to create the structure of test subfolders. <br>
* Turn on the robot, go to the next step while becomes up and running.<br>
* Open Terminal1:<br>
    * Load Anaconda environment:<br>
        * `$ source activate environment`<br>
    * Load ROS environment:<br>
        * `$ source $HOME/catkin_ws/devel/setup.bash`<br>
        * `$ export ROS_MASTER_URI=http://<ROBOT_IP>:11311`<br>
    * Once the robot is up and running:
        * `$ cd $HOME/catkin_ws/src/mic2wav/launch/`<br>
        * `$ gedit mic2wav_params.yaml`<br>
        * There are several things to look at:<br>
            * The topic where the robot publishes the raw audio it captures is already set (`/pepper_robot/audio`). If another device is going to be used, the topic can be set changing the value of the parameter called `raw_audio_topic`.<br>
            * The destination folder can be changed using the  `sound_path` parameter. Each recording session should be done repeating several sentences, using the same voice emotion. After that it will depend on the tagger to place the audio recording made in the train or test subfolder corresponding to the emotion used during the recording.<br>
            * There are other parameters that can be changed, for instance `dest_num_channels` which is the number of channels for the desired conversion, `dest_rate` which is the rate for the desired conversion or `max_iter` which sets the number of iterations after which the sound will be dumped into a WAV file.<br>
            * Once we're done with the tweaking, let's capture some audios!<br>
    * `$ roslaunch mic2wav mic2wav.launch`<br>
    * At the end of the process there should be as many folders as emotions. All these folders under the same parent folder.<br>
    * Now it's time to split all the pictures (the original and the distorted ones) into TRAIN/TEST set.<br>
