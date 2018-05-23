# Autonomous Hider
Rovio Wowwee Autonomous Hiding Algorithm

# How to run
1. Setup the rovio connection (connect hotspot, setup static IP address, gateway, etc.) 
2. By the default, the IP address in the code is `192.168.43.18`, it can be changed in `main.py`, changing value of host to the IP address that has been setup.
3. Install python requirement by running `pip install -r requirements`
4. Create a folder `ssdweights`, put the weight file `rovio_v2.h5` into folder `ssdweights`
5. To run the hiding algorithm, run `python main.py`
6. To stop the hiding algorithm, simply `ctrl + c` will do.

# Reference
[1] https://github.com/pierluigiferrari/ssd_keras

[2] https://github.com/thearn/pyrovio

# Github link
[1] https://github.com/kamwoh/autonomoushider
