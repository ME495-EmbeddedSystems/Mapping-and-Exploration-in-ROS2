# `nubot_nav` package

The `nubot_nav` package enables mapping and exploration for the [`nubot`](https://github.com/m-elwin/nubot) through the `slam_toolbox` and `nav2` stack.

***The intention behind doing this project was to gain familiarity with the `slam_toolbox` package and `nav2` stack.***

## QuickStart Guide

### Launch manual explorer
The manual exploration allows the user to move the nubot in `gazebo` (using the '`teleop_twist_keyboard` package or by giving 2-D goal poses) to explore its 2-D environment while building a map of it.

To launch manual exploration:
```bash
ros2 launch nubot_nav manual_explore.launch.xml
```

https://private-user-images.githubusercontent.com/59332714/289257687-dd179d2b-3a36-4d72-a3e6-afa457a9d8cd.mp4?jwt=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTEiLCJleHAiOjE3MDIxMDk5MDQsIm5iZiI6MTcwMjEwOTYwNCwicGF0aCI6Ii81OTMzMjcxNC8yODkyNTc2ODctZGQxNzlkMmItM2EzNi00ZDcyLWEzZTYtYWZhNDU3YTlkOGNkLm1wND9YLUFtei1BbGdvcml0aG09QVdTNC1ITUFDLVNIQTI1NiZYLUFtei1DcmVkZW50aWFsPUFLSUFJV05KWUFYNENTVkVINTNBJTJGMjAyMzEyMDklMkZ1cy1lYXN0LTElMkZzMyUyRmF3czRfcmVxdWVzdCZYLUFtei1EYXRlPTIwMjMxMjA5VDA4MTMyNFomWC1BbXotRXhwaXJlcz0zMDAmWC1BbXotU2lnbmF0dXJlPTc4OWIzMDMzYzAyZGQzZjYyZWYxY2I0ZjYxMDZjY2JhNTFkZTc2MWUyMmNkNjE3NGYxMzMwMzQ1NWJiNDQ3Y2ImWC1BbXotU2lnbmVkSGVhZGVycz1ob3N0JmFjdG9yX2lkPTAma2V5X2lkPTAmcmVwb19pZD0wIn0.8HuGUn2BdJHBVtpSZgbOazXsJE7JP6u2o4mTWum654o

### Launch autonomous explorer
The autonomous explorer node (nicknamed `dora` given the efficacy of the algorithm) explores the frontiers of the map by using what is essentially a naked (single-branched) Rapidly exploring Random Tree. The nubot is given random goal poses within its immediate vicinity to slowly map the entire region after a sufficiently long duration. This algorithm draws inspiration from Finding Nemo's Dory's philosophy: ["Just keep swimming"](https://www.youtube.com/watch?v=zya40MmN9I4), and the [`crazy_turtle`](https://github.com/m-elwin/crazy_turtle) package. As long as the nubot does not stop, it should eventually map the entire region.

To launch autonomous exploration:
```bash
ros2 launch nubot_nav explore.launch.xml
```

https://private-user-images.githubusercontent.com/59332714/289257802-e06430f5-3aef-4fb9-810c-428a42f04882.mp4?jwt=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTEiLCJleHAiOjE3MDIxMTAwNTYsIm5iZiI6MTcwMjEwOTc1NiwicGF0aCI6Ii81OTMzMjcxNC8yODkyNTc4MDItZTA2NDMwZjUtM2FlZi00ZmI5LTgxMGMtNDI4YTQyZjA0ODgyLm1wND9YLUFtei1BbGdvcml0aG09QVdTNC1ITUFDLVNIQTI1NiZYLUFtei1DcmVkZW50aWFsPUFLSUFJV05KWUFYNENTVkVINTNBJTJGMjAyMzEyMDklMkZ1cy1lYXN0LTElMkZzMyUyRmF3czRfcmVxdWVzdCZYLUFtei1EYXRlPTIwMjMxMjA5VDA4MTU1NlomWC1BbXotRXhwaXJlcz0zMDAmWC1BbXotU2lnbmF0dXJlPTNkMWFhMTRkNzFhZWI3YTYzNzVmYmQ3MTEzOGQwOGJiNTQzNTkxYjY3Zjk3OGM5OTkzOWMzNGNmOGMwNmYxZjgmWC1BbXotU2lnbmVkSGVhZGVycz1ob3N0JmFjdG9yX2lkPTAma2V5X2lkPTAmcmVwb19pZD0wIn0.JgstXCSNFSZwQegAIhBj8A5FQW975Ulq0xrUcQn843o

Author: [Aditya Nair](https://github.com/GogiPuttar)
