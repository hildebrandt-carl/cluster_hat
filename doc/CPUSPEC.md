# Installing CPU_SPEC 2006

Instructions found at:
[CPU Spec 2006 Install](http://www.spec.org/cpu2006/Docs/install-guide-unix.html)

## Installing on a Pi

SSH into the 
```
$ ssh pi@10.211.133.124
```

Get the CPU Benchmark Test
```
$ wget https://www.dropbox.com/s/axsprnixv5i7bzm/cpu2006.zip?dl=1
```

The file downloaded might be named:
```
$ cpu2006.zip?dl=1
```

If this is the case rename it:
```
$ mv cpu2006.zip?dl=1 cpu2006.zip
```

Unzip The benchmark software
```
$ unzip cpu2006.zip
```

Delete any MACOS files created by the zip
```
$ rm -rf _MACOSX/
```

Mount the ISO file
```
$ sudo mount -o loop cpu2006-1.2.iso /media/iso
```

Make a folder which you can install the software from
```
$ cd ~
$ mkdir cpuFiles
```

Go the file
```
$ cd /media
```

Copy the content of the file to home
```
$ cp -r iso/ ~/cpuFiles
```

Go into the src files
```
$ cd ~/cpuFiles/iso/tools/src
```

Make the build tools command executable
```
$ chmod a+x buildtools
```

Run the build tool
```
$ sudo FORCE_UNSAFE_CONFIGURE=1 ./buildtools
```

It would seem that this software was made and complied on much older versions of gcc. So long ago that the “gets” function in C/C++ was replaced with “fgets. The good news is that you can tell the compiler to ignore this because CPU2006 does not use the gets function anywhere. 

Removing the gets error:
```
$ cd tar-1.25/gnu/
$ sudo nano stdio.in.h
Comment out line 147

$ cd ../..

$ cd specsum/gnulib/
$ sudo nano stdio.in.h
Comment out line 162
```

Check you have removed the correct lines
```
$ cd ../..
(You should now be in ~/cpuFiles/iso/tools)
$ grep -rnw "_GL_WARN_ON_USE (gets,”
```

The output should be:
```
src/tar-1.25/gnu/stdio.in.h:147://_GL_WARN_ON_USE (gets, "gets is a security hole - use fgets instead");
src/tar-1.25/mingw/stdio.h:459:_GL_WARN_ON_USE (gets, "gets is a security hole - use fgets instead");
src/specsum/win32/stdio.h:474:_GL_WARN_ON_USE (gets, "gets is a security hole - use fgets instead");
src/specsum/gnulib/stdio.in.h:162://_GL_WARN_ON_USE (gets, "gets is a security hole - use fgets instead”);
```

(Notice the // on line 147 and 162)

Build the tools
```
$ sudo FORCE_UNSAFE_CONFIGURE=1 ./buildtools
```


## Installing on a VM with Ubuntu 16.04

Insert the CD
This can be done by mounting the ISO into your VM.

Install the software
```
$ ./install
```
(Make sure you dont sudo)
Select where you want to install it (I suggest ~/cpu2006)

Change your current directory to the top-level SPEC directory and source shrc
```
$ cd ~/cpu2006/
$ . ./shrc
```

Test that you can run a benchmark using the real input set - the "reference" workload. For example:
```
$ cd config
$ cp Example-linux64-amd64-gcc43+.cfg mytest.cfg
$ runspec --config=mytest.cfg --action=build --tune=base bzip2
```

Build successes: 401.bzip2(base)          <<-- what we want to see

```
$ runspec --config=mytest.cfg --size=test --noreportable --tune=base --iterations=1 bzip2
```

Success: 1x401.bzip2                           <<-- what we want to see

Looking at the results
```
$ cd result
$ cat CINT2006.003.ref.txt 
```