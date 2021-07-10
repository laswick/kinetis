
# README

## Freescale Kinetis K60 / ARM Cortex M4 Project

Rob Laswick  
April 21 2012    
[www.laswick.net](http://www.laswick.net)

---

> **Project Status: Inactive**

> This project was started in March 2012 and ended in May 2013.  This repository will remain available but is no longer maintained.

---

## Welcome!

Hopefully you'll find the information in this project helpful and useful.  All source code in this project is freely available to be used however you like, for learning purposes, and/or to be used in part or in full, in both open and closed source projects.  See the `COPYING` file for more information.

All the files and source code in this project were developed and made available during the process of a long term Problem Based Learning ([PBL](http://en.wikipedia.org/wiki/Problem-based_learning)) style after-hours project at [L-3 Wescam](http://www.wescam.com) by volunteers from the Software and Electrical Engineering departments.

>#### PBL IN A NUTSHELL

>_A highly successful and enjoyable way to learn something new, and engage others in the process is to first determine what exactly it is you what want to learn, then to develop a road map of what's required to effectively meet that goal.  Once the road map has been established, items/topics on the road map are delegated to each of the participants.  Everyone is responsible for learning their assigned topic, then later teaching it to the group.  As well as being a highly effective way to learn new things, there are so many other benefits to this method, one of the most noteworthy being a sense of teamwork and knowledge sharing.  This approach has proven to be a very successful method of team building in the work place._


The goal of this project was to take a group of talented embedded systems engineers on a soul searching journey to develop a **detailed** yet **practical** understanding of the ARM Cortex-M4 using the Freescale Kinetis family of microcontrollers, the super cool K60 Tower development kits, and the free CodeSourcery GNU tools.  The project was broken down into 3 main phases, with each phase consisting of a number of relevant topics.  Everyone participating was assigned one or more topics per phase that they were responsible for learning, and later teaching to the rest of the team, based on an predefined presentation schedule. Prior to the presentation, everyone is responsible for developing a set of software corresponding to their topic (Makefile, driver, and demo, etc.) and commit it to the project repository.  A major portion of each presentation was spent stepping through every line of code and clearly explaining the what and whys.

---

## PHASE 1: Assembly Programming

This phase focuses strictly on understanding the fundamentals of how an ARM Cortex M4 processor actually works, and ARM assembly level programming (including writing processor specific start code, linker scripts, and Makefiles from scratch!).  Far too often developers and engineers riddle their resumes with lists of all the processors they've worked with, but in reality most if not all of them code almost exclusively in a higher level language (i.e. C/C++) which abstracts out almost all of the low level details!  Higher level languages definitely have their advantages, but let's be honest, if you only code in C it's very difficult to tell one processor from another.  The bottom line is there is value in understanding how a processor "really" works under the hood.

_This phase started on April 12 2012 and completed on June 7 2012._

---

## PHASE 2: Embedded C Programming

This phase focuses on "embedded" C programming and how it differs from traditional (non-embedded) C programming.  We'll be diving further into start code and linker scripts, with an emphasis on the infamous crt0: C runtime environment setup (i.e. "getting to main()" -- you'd be surprised how many so called embedded "experts" don't know what goes on before main() is called!  We unveil this great mystery).  This phase also focuses on developing generic full featured drivers, with working demos for almost all of the Freescale peripherals attached to the ARM core in the K60.

We also choose to explore the POSIX device driver model (i.e. open, close, read, write, ioctl), which we completely understand can be total overkill for typical low level, non-RTOS based applications, but for the purpose of a learning exercise, this project seemed like a good opportunity to dive into the details of implementing such an interface.

_This phase started on June 21 2012, with a summer break, and completed
on November 22 2012._

---

## PHASE 3: Real Time Embedded C Programming

This phase focuses on getting a real time operating system running from the ground up, and writing "thread-safe" low level drivers.  We'll be using the Chibi RTOS, which is a very impressive full featured open source RTOS.

_This phase started on January 24 2013, and completed on February 21 2013._

---

If you have any questions, comments, concerns, or ... in the extremely rare event you find a, _(gasp)_ software bug, please feel free to reach out to me via email at robert.laswick@gmail.com.

