# CityBrainChallenge-starter-kit

A alternative version which can run on your own machine without docker.

(Updated 20210506)

## Description

This is a alternative of the formal edition, which you can run on your own machine directly.
The CBEngine directory contents the Formal env, and file citypb.cpython-37m-x86_64-linux-gnu.so is a compiled lib too.
This repository works well on my machine. if you don't want your code looks ugly you can also move CBEngine and citypd* to your local python libary path,such as "/usr/local/lib/python3.7/dist-packages" for linux

## Notice

By the way , I just test it on linux platform. So if you run it on windows you may should test it yourself.


### FAQ

1. 
```
IndexError: basic_string::at: __n (which is 18446744073709551615) >= this->size() (which is 2)
```

If your operating system is Windows and you have set `git config --global core.autocrlf true` , when you clone this repo, git will automatically add CR to cfg/simulator.cfg. This will lead to the error in Linux of the docker container.

So please change cfg/simulator.cfg from CRLF to LF after cloning this repo.