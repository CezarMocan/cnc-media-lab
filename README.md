## Shadow Network

A networked Kinect + openFrameworks + MaxMSP software for two remote performers, which turns body movements into sound. 

**Further project details, documentation and demos can be found [here](http://cezar.io/shadownetwork).**

![Shadow Network](https://i.imgur.com/ifdLtl3.png)

**Project Context**

* Built during Summer 2020, as a team of two, through a grant from the New York City Media Lab. I was involved in concept development, visual design and openFrameworks programming. My teammate worked on concept development, sound design and MaxMSP programming.
* The total project development timeline was 10 weeks, at 30-40 hours a week, from accepted proposal to functional demo. We had originally proposed a physical installation, but had to adapt our project to a networked setup because of COVID. 
* The current form of the project emerged somewhere around weeks 4-5, after doing a number of visual & sonic experiments, meaning that most of the code in this repository came together during 5-6 weeks.


**Technical Information**

* This repository only contains the C++ code belonging to the openFrameworks part of the installation. The MaxMSP patches are not uploaded here yet.
* Below is a technical diagram of the entire project, for better context:
![Technical Diagram](http://cezar.io/assets/images/shadownetwork/2.png)
* The openFrameworks code is responsible for getting input from the Kinect for the two players, for passing body & movement parameters to MaxMSP for sound generation, for coordinating the network communication between the two players, and for generating & rendering the visuals of the project.
* I used Visual Studio as an IDE for development, and the internal file organization structure from VS isn't reflected on the file system, so navigating the repository might be a bit confusing. For reference, the VS project is organized like this:

![Visual Studio Structure](https://i.imgur.com/IZn9E2B.png)

* The C++ in this project (which is a pretty accurate reflection of my skill) is quite rudimentary. Even though I've learned programming with C++ (was using it for competitive programming, between 2005 and 2012) and choose openFrameworks for any creative coding project or installation that's too much for Javascript, I've only written C++ "professionally" – as part of a team, with standards, code reviews & co. – on a 3-month long project in 2012. Meaning that I can be productive & deliver from day one, but might need a bit of time to learn better patterns, newer language features & so on.
