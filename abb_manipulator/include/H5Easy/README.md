# H5Easy
---------
Simple implementations of HDF5 Reading and Writing

The purpose of this is to provide an easier method to read and write from hdf5 formats. Currently the program works with  
int, float, and double datatypes. The program will auto detect if you are trying to read or write in integer or float and
properly store the data as such. With just a few lines you can read or write data easily. 

##### H5Easy allows you to
- Write type T, vector<T>, vector<vector<T> >
- Load single, vector, and 2D vector ints, floats, and doubles
- Get size of a dataset (note for 2D data sets it'll show number of total elements)

## How to use
-------------
The classes here are simple to use and reduce your reading and writing of h5 files to just a few lines. The intention was to mimic the behaviour 
of Python's h5py. With just a few lines you can quickly grab or write to h5 files. 

To compile just make sure that you have `h5rw.h` in your `LD_LIBRARY_PATH`. If you are on linux run `export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/path/to/h5rwDir` 
or just place it in `/usr/include`. To compile you MUST use an h5 compiler, such as `h5c++`. 

If a user wants to write to a h5 file all they need to do is run the following
```
WriteH5 data;
data.setFileName("myH5File.h5");
data.setVarName("myVariableName");
data.writeData(myVectoredData);
// For groups
data.createGroup("/my/group/name");
data.setVarName("/my/group/name/myVariableName");
```
The libraries do not care if you pass it a float or an int vector, just that you pass it a vector. It will write in the proper method.

If a user wants to read the data from the h5 file
```
LoadH5 data;
data.setFileName("myH5File.h5"); // Set file name (only need once)
data.setVarName("myVariableName"); // Set variable name
vector<type> loadedData = data.getData(); // Load the data
// For groups
data.setVarName("/my/group/name/myVariableName");
vector<type> loadedData = data.getData();
// Other features
data.getSize(); // Get the size of variable (/my/group/name/myVariableName)
```
If you are trying to read a double as a float you will get an error. If you wish to cast you 
should do so after the data load

I have included a test file so that you can ensure that things are working properly.
Note that this method also works for loading in group data. All you have to do is type the full path as the variable name.

If you are returned with a vector size 1 with only the entry of -1 you have gotten an error.

### Assumptions
---------------
- You are using vectors or scalar values (int, float, double)
- You are using 1D or 2D vectors (reading and writing)
- You are reading "STD_I{8,16,32,64}{LE,BE}" or "IEEE_F{32,64}{LE,BE}" data (eg STD_I32LE)
- You are compiling with `-std=c++11` or higher. 

If you are unsure of the data type you are reading run `h5dump -H filename.h5`. This will show you
the data type. Make sure it matches one of the above.

### ISSUES
-----------
###### Data written is garbage
- Check output from `h5dump` to make sure that it is really garbage. 
- Check that you aren't casting when writing. Cast before. 
###### Data returned is garbage
- Check output from `h5dump` and ensure that data was written correctly.
- Check that you are returning the correct type. You cannot cast data during read. Casting must be done after data is read.
###### Libraries not linking:
 - Make sure hdf5-dev is installed. 
 - Check that the libraries are being linked properly. They should be located in `/usr/include` but if you can't find them then run `sudo find /usr -name hdf5` or `sudo find /usr -name H5Cpp.h`. 
   - Test that this is the issue by running `h5c++ -I/path/that/you/found test.cpp -o test` If this works then add it to your LD_LIBRARY_PATH. `export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/path/that/you/found`. Add that to your `.bashrc` or shell's rc file. 
##### Code won't compile
   - If you have Anaconda (python) installed DO NOT use its `h5c++`. If you are still getting library problems check the output of `which h5c++`. It should point to `/usr/bin/h5c++`, if you installed it through apt
   - Check that you are using `/usr/bin/h5c++`

###### h5dump not working:
 - Make sure that hdf5-tools is installed
If you can not find these through your package manager just search for `hdf5`


### Dependencies
----------------
- h5 libraries and its dependencies
- libhdf5-dev
- hdf5-tools
- h5utils
- libpthread.so.0
- libsz.so.2
- libz.so.1
- libdl.so.2
- libstdc++.so.6
- libm.so.6
- libgcc_s.so.1
- libc.so.6

### Problems / Feature Requests
--------------
Please open an issue on the GitHub page for bugs in the code or feature requests. I will try to address everything as best as I can

### TODO
--------
Features I plan on adding
- cast doubles to floats and vise versa
- Other data types (uints, chars, objects, others. Please request)
- N-dimensional arrays 
- More generalized code
