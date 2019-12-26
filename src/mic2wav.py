#!/usr/bin/env python
# license removed for brevity
import alsaaudio
import librosa
import numpy as np
import os
import rospy
import struct
import time
from naoqi_bridge_msgs.msg import AudioBuffer
from subprocess import call
from std_msgs.msg import String
 
class Mic2Wav():
    def __init__(self):
        # To control the number of iterations
        self.counter = 0
        print(self.counter)
        # To accumulate the incoming bytes for each iteration
        self.dataBuff = None

        # Parameters are loaded from ../launch/mic2wav_params.yaml
        (raw_audio_topic, self.dest_num_channels, self.dest_rate, self.max_iter, self.sound_path, self.wav_topic) = self.get_parameters()

        # Device is configured
        rospy.loginfo("Getting audio card...")
        self.device = alsaaudio.PCM()
        self.device.setformat(alsaaudio.PCM_FORMAT_S16_LE)
        rospy.loginfo("Done!")

        # Check whether the dir to store the processed sound exists
        if not (os.path.exists(self.sound_path)):
            os.makedirs(self.sound_path)

        # Let's subscribe to the topic where the robot publishes audio
        sub_sound = rospy.Subscriber(raw_audio_topic, AudioBuffer, self.speechCB, queue_size=1)    
        # The name of the file containing the processed sound is published
        self.pub_wav_name = rospy.Publisher(self.wav_topic, String, queue_size=1)
        rospy.spin()

    def genHeader(self, sampleRate, bitsPerSample, channels, samples):
        datasize = len(samples) * channels * bitsPerSample // 8
        #datasize = len(samples) * bitsPerSample // 8
        o = bytes("RIFF").encode('ascii')                                      # (4byte) Marks file as RIFF
        #o += to_bytes(datasize + 36, 4,'little')                              # (4byte) File size in bytes excluding this and RIFF marker
        o += struct.pack('<I',datasize + 36)                                   # (4byte) File size in bytes excluding this and RIFF marker
        o += bytes("WAVE").encode('ascii')
        o += bytes("fmt ").encode('ascii')                                     # (4byte) Format Chunk Marker
        #o += to_bytes(16, 4,'little')                                         # (4byte) Length of above format data
        o += struct.pack('<I',16)                                              # (4byte) Length of above format data                     
        #o += to_bytes(1, 2,'little')                                          # (2byte) Format type (1 - PCM)
        o += struct.pack('<H',1)                                               # (2byte) Format type (1 - PCM)
        #o += to_bytes(channels, 2,'little')                                   # (2byte)
        o += struct.pack('<H',channels)                                        # (2byte)
        #o += to_bytes(sampleRate, 4,'little')                                 # (4byte)
        o += struct.pack('<I', sampleRate)                                     # (4byte)
        #o += to_bytes((sampleRate * channels * bitsPerSample) // 8, 4,'little')# (4byte)
        o += struct.pack('<I',(sampleRate * channels * bitsPerSample) // 8)    # (4byte)
        #o += to_bytes((channels * bitsPerSample) // 8, 2,'little')            # (2byte)
        o += struct.pack('<H', (channels * bitsPerSample) // 8)                # (2byte)
        #o += to_bytes(bitsPerSample, 2,'little')                              # (2byte)
        o += struct.pack('<H', bitsPerSample)                                  # (2byte)
        o += bytes("data").encode('ascii')                                     # (4byte) Data Chunk Marker
        #o += to_bytes(datasize, 4,'little')                                    # (4byte) Data size in bytes
        o += struct.pack('<I',datasize)                                    # (4byte) Data size in bytes

        return o

    def get_parameters(self):
        """
        Gets the necessary parameters from parameter server. See ../launch/mic2wav_params.yaml

        """

        raw_audio_topic = rospy.get_param("~raw_audio_topic")
        dest_num_channels = rospy.get_param("~dest_num_channels")
        dest_rate = rospy.get_param("~dest_rate")
        max_iter = rospy.get_param("~max_iter")     
        sound_path = rospy.get_param("~sound_path")
        wav_topic = rospy.get_param("~wav_topic")

        return (raw_audio_topic, dest_num_channels, dest_rate, max_iter, sound_path, wav_topic)


    def speechCB(self, msg):
        rospy.loginfo("Callback received!")
        print(len(msg.channelMap))
        print(msg.frequency)
        print(len(msg.data))
        # Number of channels and frequency are set
        self.device.setchannels(len(msg.channelMap))
        self.device.setrate(msg.frequency)


        # The current audio sequence is processed and appended to what's gathered in dataBuff so far
        tmp = np.array(list(msg.data)).reshape(-1, 4)
        if (self.dataBuff is not None):
            self.dataBuff = np.vstack((self.dataBuff, tmp))
        else:
            self.dataBuff = tmp            
    
        print(self.counter)
        # Only when max_iter is reached, the audio processing occurs
        if (self.counter == self.max_iter):
            print("SHAPE")
            print(self.dataBuff.shape)
            print(len(self.dataBuff))
            timestamp = str(time.time())

            filename_raw = os.path.join(self.sound_path,"raw_" + timestamp + ".wav")            
            f = open(filename_raw, 'wb')
            #f.write(self.genHeader(48000, 16, 4, self.dataBuff))
            header = self.genHeader(48000, 16, 4, self.dataBuff)
            body = ""
            bodyElements = None
            channels = range(4)
            for sample in self.dataBuff:                
                for c in channels:     
                    bodyChannel = []
                    s = sample[c]
                    sf = float(s) / float(32768)
                    if(sf>1):
                        sf = 1
                    if(sf<-1):
                        sf = -1

                    bodyChannel.append(sf)
                    body += struct.pack('<h', int(sf*32767))
                    #f.write(struct.pack('<h', int(sf*32767)))
                    #print(sf)
                    #if bodyElements is not None:
                    #    bodyElements = np.vstack((bodyElements, np.array(bodyChannel)))
                    #else:
                    #    bodyElements = np.array(bodyChannel)

            
            print("Saving array")
            self.dataBuff = np.true_divide(self.dataBuff.astype(np.float32), float(32768))
            np.save(os.path.join(self.sound_path,"nparray" + timestamp + ".npy"), self.dataBuff)
            self.dataBuff = np.where(self.dataBuff < -1.0 , -1.0, self.dataBuff)
            self.dataBuff = np.where(self.dataBuff > 1.0, 1.0, self.dataBuff)


            f.write(header + body)
            f.close()
            self.dataBuff = None
            self.counter = 0
            #print("BODY ELEMENTS LENGTH " + str(bodyElements.shape))
            self.pub_wav_name.publish(header + body)
        else:
            self.counter += 1
 
if __name__ == '__main__':
    try:
        rospy.init_node('mic2wav', anonymous=False)
        m2w = Mic2Wav()
    except rospy.ROSInterruptException:
        pass

