
Primary mediapipe info:
https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker

That links to a repo with examples that were kinda helpful; some of it is in draw.py
I hadn't gotten around to setting up a virtual environment yet; you'll want to do that because we'll all need to do that anyways. No checking .venv folder into git; there's a freeze method where we treat it like it's package-lock.json from node. If that doesn't mean anything to you, do some searching for information about using python virtual environments with git.

This article was also helpful and I'm currently working on the modules of mediapipe it uses, but I'm not sure it's what we want long-term:
https://bleedaiacademy.com/introduction-to-pose-detection-and-basic-pose-classification/

In the meeting Monday he talked about how we probably want to do the model detection where you make the detection within the context of all prior frames & interpolate missing ones (if a joint is obstructed, for example). The mediapipe detect_async for live video (see first link) does that, but I wasn't able to get it working with OpenCV. Take a look at it yourselves; let me know if you have questions

Files:
- `first.py` -> mediapipe with a given image, in this case `tkd.jpeg`
- `second.py` -> attempts to use `detect_async` with live feed and cv2; play with this one
- `third.py` -> in progress work on using matplotlib per the second link above
- `videocapture.py` -> test file to see that opencv is reading your webcam successfully; there's lots of articles on doing so if it gives you trouble
- `draw.py` -> largely from the mediapipe github examples; has function to draw lines for detected points/edges onto the given image