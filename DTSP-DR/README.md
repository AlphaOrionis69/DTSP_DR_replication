1. How to download SCIP solver
   - You can download from https://www.scipopt.org/ and try to install it via Cmake, you must have a folder with 3 subfolders named: bin, include, lib in the end.

   - You can also download directly that folder here: https://drive.google.com/drive/folders/1mc-wFAVbxJa2Fs2u4l1pOBpczEQPrUJ-?usp=drive_link
   
   
2. How to run in CodeBlocks (assuming that you have the folder mentioned above)
   Suppose your final folder mentioned above has the path C:\scip
   - In CodeBlocks, create a new project

   - Create and copy the file (C source file, C header file and Instances folder) into your project, then try to build and run it

   - If not, then you should do the followings:
      + In CodeBlocks, you would see a "Project" menu, click on it and choose "Build options", a window would be opened
      + Make sure that you are at the root of the project tree, you can check it by looking at a small tree in the left side of the window
      + After that, you click on "Linker settings", then add in the "Link libraries" box all the names of lib file inside the folder C:\scip\lib (note that you only add the names without the .lib)
      + Still in the window, you click on "Search directories", and make sure in "Search directories", you are in "Compiler", then in the big box below, you add the path "C:\scip\include"
      + Finally, move to "Linker" right beside "Complier" and then add the path "C:\scip\lib" in the box below, after that click OK and you can run the program.

   - If your code still not work, you can copy all the .dll file (which is at C:\scip\bin) into your bin project folder
