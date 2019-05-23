/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 Steven Walton
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.  
 *
 *
 * Class functions for an easier h5 read and write method.
 * Created by: Steven Walton
 * Email: walton.stevenj@gmail.com
 * Version: 1.2
 *
 * If you wish to contribute to this endeavour please email me or fork my branch from
 * https://github.com/stevenwalton/H5Easy
 * 
 */
#ifndef H5RW_H
#define H5RW_H

#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <typeinfo>

#include "H5Cpp.h"

using namespace H5;

class WriteH5
{
   private:
      std::string variable;
      std::string filename;
   public:
      // sets our filename and our variable name
      void setFileName ( std::string name ) {filename = name;};
      void setVarName  ( std::string name ) {variable = name;};
      // Functions to be overloaded
      template<typename T>
      void writeData(const std::vector<T>&);
      template<typename T>
      void writeData(const std::vector<std::vector<T> >&);
      template<typename T>
      void writeData(const T&);

      void createGroup(std::string);
};

class LoadH5
{
   private:
      std::string variable;
      std::string filename;
   public:
      // sets our filename and our variable name
      void setFileName(std::string name) {filename = name;};
      void setVarName(std::string name) {variable = name;};
      // Read functions
      int getDataint() const;
      float getDatafloat() const;
      double getDatadouble() const;
      std::vector<int> getDataVint() const;
      std::vector<float> getDataVfloat() const;
      std::vector<double> getDataVDouble() const;
      std::vector<std::vector<int> > getData2Dint() const;
      std::vector<std::vector<float> > getData2Dfloat() const;
      std::vector<std::vector<double> > getData2Ddouble() const;
      // Return the size of the data
      // Note that for multi-dim arrays that it gets the total size and not the size of a single row.
      int getSize() const;

      // We now make a proxy class so that we can overload the return type and use a single
      // function to get data whether int or float. This could be made more advanced by 
      // adding more data types (such as double). 
      class Proxy
      {
         private:
            LoadH5 const* myOwner;
         public:
            Proxy( const LoadH5* owner ) : myOwner( owner ) {}
            operator int() const
            {
                return myOwner->getDataint();
            }
            operator float() const
            {
                return myOwner->getDatafloat();
            }
            operator double() const
            {
                return myOwner->getDatadouble();
            }
            operator std::vector<int>() const
            {
               return myOwner->getDataVint();
            }
            operator std::vector<float>() const
            {
               return myOwner->getDataVfloat();
            }
            operator std::vector<double>() const
            {
               return myOwner->getDataVDouble();
            }
            operator std::vector<std::vector<int> >() const
            {
               return myOwner->getData2Dint();
            }
            operator std::vector<std::vector<float> >() const
            {
               return myOwner->getData2Dfloat();
            }
            operator std::vector<std::vector<double> >() const
            {
               return myOwner->getData2Ddouble();
            }
      };
      // Here we use the Proxy class to have a single getData function
      Proxy getData() const {return Proxy(this);}
};



/*
 * ************************************************************************************************
 * ************************************ Write Functions *******************************************
 * ************************************************************************************************
 */

// Numeric implementation of our write data function
// Only accepts numerical values. Integers, floats, or doubles

template<typename T>
void WriteH5::writeData(const T &data) 
{
    Exception::dontPrint();
    uint itr = 0;
    auto *a = new T{data};
    char* type = (char*)(typeid(T).name());
    int vrank = 0;
    hsize_t dims[1];
    dims[0] = 1;
    const H5std_string FILE_NAME(WriteH5::filename);
    H5std_string DATASET_NAME(WriteH5::variable);
    while (true)
    {
        try
        {
            H5File file(FILE_NAME, H5F_ACC_RDWR);
            DataSpace dsp = DataSpace(vrank, dims);
            // int
            if ( type == (char*)typeid(int).name() )
            {
                DataSet dset = file.createDataSet(DATASET_NAME, PredType::STD_I32LE, dsp);
                dset.write(a, PredType::STD_I32LE);
                dset.close();
            }
            // float
            else if ( type == (char*)typeid(float).name() )
            {
                DataSet dset = file.createDataSet(DATASET_NAME, PredType::IEEE_F32LE, dsp);
                dset.write(a, PredType::IEEE_F32LE);
                dset.close();
            }
            else if ( type == (char*)typeid(double).name() )
            {
                DataSet dset = file.createDataSet(DATASET_NAME, PredType::IEEE_F64LE, dsp);
                dset.write(a, PredType::IEEE_F64LE);
                dset.close();
            }
            else
            {
                std::cout << "Unknown data type! EXITING" << std::endl;
                exit(1);
            }
            dsp.close();
            file.close();
            delete a;
            break;
        }
        catch (FileIException error)
        {
            H5File file(FILE_NAME, H5F_ACC_TRUNC);
            file.close();
            itr++;
            if ( itr > 3) 
            {
                std::cout << "We've tried too many times in the Int writing sequence" << std::endl;
                break;
            }
        }
    }
}

template<typename T>
void WriteH5::writeData(const std::vector<T> &data)
{
   Exception::dontPrint();

   uint itr = 0; // Used to ensure we don't get stuck in an infinite loop
   uint npts = data.size(); // size of our data
   auto *a = new T[npts]; // convert to an array
   char* type = (char*)(typeid(a[0]).name());
   int vrank = 1; // since we are using std::vectors we are storing everything in one dimension

   // convert std::vector to array. H5 does not seem to like the pointer implementation
   for (size_t i = 0; i < npts; ++i)
      a[i] = data[i];
   // conventional syntax for H5 data writing
   hsize_t dims[1];
   dims[0] = npts;
   // Let's make sure we are doing what we want and output it to the std output

   // We need to set these parameters for the H5 data file writing
   const H5std_string FILE_NAME(WriteH5::filename);
   H5std_string DATASET_NAME(WriteH5::variable);
   // loop here will check if the file exists. 
   while (true)
   {
      // This assumes that the file already exists and will then write to the file
      try
      {
         H5File file(FILE_NAME, H5F_ACC_RDWR);
         DataSpace dsp = DataSpace(vrank,dims);
         // int
         if ( type == (char*)typeid(int).name() )
         {
            DataSet dset = file.createDataSet(DATASET_NAME, PredType::STD_I32LE, dsp);
            dset.write(a, PredType::STD_I32LE);
            dset.close();
         }
         // uint
         else if ( type == (char*)typeid(uint).name() )
         {
            DataSet dset = file.createDataSet(DATASET_NAME, PredType::STD_U32LE, dsp);
            dset.write(a, PredType::STD_U32LE);
            dset.close();
         }
         // float
         else if ( type == (char*)typeid(float).name() )
         {
            DataSet dset = file.createDataSet(DATASET_NAME, PredType::IEEE_F32LE, dsp);
            dset.write(a, PredType::IEEE_F32LE);
            dset.close();
         }
         // double
         else if ( type == (char*)typeid(double).name() )
         {
            DataSet dset = file.createDataSet(DATASET_NAME, PredType::IEEE_F64LE, dsp);
            dset.write(a, PredType::IEEE_F64LE);
            dset.close();
         }
         else
         {
            std::cout << "Unknown data type! EXITING" << std::endl;
            exit(1);
         }

         // remember to close everything and delete our arrays
         dsp.close();
         file.close();
         delete[] a;
         break;
      }
      // Here we are catching if the file does not exist. We will then create a new file and return
      // back to the try statement
      catch (FileIException error)
      {
         H5File file(FILE_NAME, H5F_ACC_TRUNC);
         file.close();
         // Just some warning that we have gone through this catch
         itr++;
         // This is to prevent us from getting caught in an infinite loop. While (true) loops
         // are useful, but they can be dangerous. Always ensure some escape sequence. Could
         // just use a for loop
         if ( itr > 3) 
         {
            std::cout << "We've tried too many times in the Int writing sequence" << std::endl;
            break;
         }
      }
   }
}

template<typename T>
void WriteH5::writeData(const std::vector<std::vector<T> > &data)
{
   Exception::dontPrint();

   uint itr = 0; // Used to ensure we don't get stuck in an infinite loop
   uint dim1 = data.size(); // size of our data
   uint dim2 = data[0].size();
   auto a = new T[dim1*dim2]; // convert to an array
   auto md= new T*[dim1];
   for ( size_t i = 0; i < dim1; ++i )
      md[i] = a + i*dim2;

   int vrank = 2; // since we are using std::vectors we are storing everything in one dimension

   // convert std::vector to array. H5 does not seem to like the pointer implementation
   for (size_t i = 0; i < dim1; ++i)
      for ( size_t j = 0; j < dim2; ++j )
         md[i][j] = data[i][j];
   // conventional syntax for H5 data writing
   hsize_t dims[2];
   dims[0] = (int)dim1;
   dims[1] = (int)dim2;
   //hid_t memspace_id = H5Screate_simple(vrank, dims, NULL);
   // Let's make sure we are doing what we want and output it to the std output

   // We need to set these parameters for the H5 data file writing
   const H5std_string FILE_NAME(WriteH5::filename);
   H5std_string DATASET_NAME(WriteH5::variable);
   // loop here will check if the file exists. 
   while (true)
   {
      // This assumes that the file already exists and will then write to the file
      try
      {
         H5File file(FILE_NAME, H5F_ACC_RDWR);
         DataSpace dsp = DataSpace(vrank,dims);
         // int
         if ( typeid(T).name()== typeid(int).name() )
         {

            DataSet dset = file.createDataSet(DATASET_NAME, PredType::STD_I32LE, dsp);
            dset.write(a, PredType::STD_I32LE);
            dset.close();
         }
         // uint
         else if ( typeid(T).name() == typeid(uint).name() )
         {
            DataSet dset = file.createDataSet(DATASET_NAME, PredType::STD_U32LE, dsp);
            dset.write(a, PredType::STD_U32LE);
            dset.close();
         }
         // float
         else if ( typeid(T).name()== typeid(float).name() )
         {
            DataSet dset = file.createDataSet(DATASET_NAME, PredType::IEEE_F32LE, dsp);
            dset.write(a, PredType::IEEE_F32LE);
            dset.close();
         }
         // double
         else if ( typeid(T).name()== typeid(double).name() )
         {
            DataSet dset = file.createDataSet(DATASET_NAME, PredType::IEEE_F64LE, dsp);
            dset.write(a, PredType::IEEE_F64LE);
            dset.close();
         }
         else
         {
            std::cout << "Unknown data type! EXITING" << std::endl;
            exit(1);
         }

         // remember to close everything and delete our arrays
         dsp.close();
         file.close();
         delete[] md;
         delete a;
         break;
      }
      // Here we are catching if the file does not exist. We will then create a new file and return
      // back to the try statement
      catch (FileIException error)
      {
         H5File file(FILE_NAME, H5F_ACC_TRUNC);
         file.close();
         // Just some warning that we have gone through this catch
         itr++;
         // This is to prevent us from getting caught in an infinite loop. While (true) loops
         // are useful, but they can be dangerous. Always ensure some escape sequence. Could
         // just use a for loop
         if ( itr > 3) 
         {
            std::cout << "We've tried too many times in the Int writing sequence" << std::endl;
            break;
         }
      }
   }
}
void WriteH5::createGroup(std::string groupName)
{
   try
   {
      H5std_string FILE_NAME(WriteH5::filename);
      std::istringstream ss(groupName);
      std::string token;
      std::vector<std::string> groupSections;
      while ( std::getline(ss, token, '/') )
         groupSections.push_back(token);
      std::string mygroup;
      for ( size_t i = 0; i < groupSections.size(); ++i )
      {
         mygroup.append("/");
         mygroup.append(groupSections[i]);
         if ( mygroup != "/" )
         {
            H5File file(FILE_NAME, H5F_ACC_RDWR);
            Group group(file.createGroup(mygroup));
            group.close();
            file.close();
         }
      }
   }
   catch (FileIException error)
   {
      error.printError();
   }
   catch (GroupIException error)
   {
      error.printError();
   }
}

/*
 * ************************************************************************************************
 * ************************************ Read Functions ********************************************
 * ************************************************************************************************
 */

int LoadH5::getSize() const
{
    try
    {
        Exception::dontPrint();
        H5std_string FILE_NAME(LoadH5::filename);
        H5File file(FILE_NAME, H5F_ACC_RDONLY);
        DataSet dataset = file.openDataSet(LoadH5::variable);
        DataSpace dataspace = dataset.getSpace();
        const int npts = dataspace.getSimpleExtentNpoints();
        return npts;
    }
    catch (FileIException error)
    {
       error.printError();
       int err = -1;
       return err;
    }
    catch (GroupIException error)
    {
       error.printError();
       int err = -1;
       return err;
    }
}

int LoadH5::getDataint() const
{
    try
    {
        Exception::dontPrint();
        H5std_string FILE_NAME(LoadH5::filename);
        H5File file(FILE_NAME, H5F_ACC_RDONLY); // Only reads
        DataSet dataset = file.openDataSet(LoadH5::variable);
        DataType datatype = dataset.getDataType();
        DataSpace dataspace = dataset.getSpace();
        H5T_class_t classt = datatype.getClass();
        if ( classt != 0 )
        {
            std::cout << LoadH5::variable << " is not an int... you can't save this as an int." << std::endl;
            exit(1);
        }
        int *data = new int;
        IntType itype = dataset.getIntType();
        H5std_string order_string;
        FloatType ftype = dataset.getFloatType();
        H5T_order_t order = ftype.getOrder( order_string);
        size_t size = ftype.getSize();
        if ( order == 0 && size == 1 )
           dataset.read(data, PredType::STD_I8LE); // Our standard integer
        else if ( order== 0 && size == 2 )
           dataset.read(data, PredType::STD_I16LE); // Our standard integer
        else if ( order== 0 && size == 4 )
           dataset.read(data, PredType::STD_I32LE); // Our standard integer
        else if ( order== 0 && size == 8 ) 
           dataset.read(data, PredType::STD_I64LE);
        else if ( order== 1 && size == 1 )
           dataset.read(data, PredType::STD_I8BE); // Our standard integer
        else if ( order== 1 && size == 2 )
           dataset.read(data, PredType::STD_I16BE); // Our standard integer
        else if ( order== 1 && size == 4 )
           dataset.read(data, PredType::STD_I32BE);
        else if ( order== 1 && size == 8 )
           dataset.read(data, PredType::STD_I64BE);
        else 
           std::cout << "Did not find data type" << std::endl;
        // Manage our memory properly
        int v = *data;
        delete data;
        dataspace.close();
        datatype.close();
        dataset.close();
        file.close();
        return v;
   }
    catch (FileIException error)
    {
       error.printError();
       int err = -1;
       return err;
    }
    catch (GroupIException error)
    {
       error.printError();
       int err = -1;
       return err;
    }
}
float LoadH5::getDatafloat() const
{
   try
   {
      Exception::dontPrint();
      //std::cout << "Getting float data" << std::endl;
      H5std_string FILE_NAME(LoadH5::filename);
      H5File file(FILE_NAME, H5F_ACC_RDONLY);
      DataSet dataset = file.openDataSet(LoadH5::variable);
      DataType datatype = dataset.getDataType();
      DataSpace dataspace = dataset.getSpace();
      H5T_class_t classt = datatype.getClass();
      if ( classt != 1 )
      {
         std::cout << LoadH5::variable << " is not a float... you can't save this as a float." << std::endl;
         exit(1);
      }
      FloatType ftype = dataset.getFloatType();
      H5std_string order_string;
      H5T_order_t order = ftype.getOrder( order_string);
      size_t size = ftype.getSize();
      float *data = new float;
      if ( order == 0 && size == 4 )
         dataset.read(data, PredType::IEEE_F32LE); // Our standard integer
      else if ( order == 0 && size == 8 ) 
      {
         dataset.read((float*)data, PredType::IEEE_F64LE);
         std::cout << "NOTE: This is actually double data. We are casting to float" << std:: endl;
      }
      else if ( order == 1 && size == 4 )
         dataset.read(data, PredType::IEEE_F32BE);
      else if ( (order_string == "Big endian byte order_stringing (1)" || order == 1) && size == 8 )
      {
         std::cout << "NOTE: This is actually double data. We are casting to float" << std:: endl;
         dataset.read((float*)data, PredType::IEEE_F64BE);
      }
      else 
         std::cout << "Did not find data type" << std::endl;
      float v = *data;
      delete data;
      dataspace.close();
      datatype.close();
      dataset.close();
      file.close();
      return v;
   }
   catch (FileIException error)
   {
      error.printError();
      float err = -1.;
      return err;
   }
   catch (GroupIException error)
   {
      error.printError();
      float err = -1.;
      return err;
   }
}
double LoadH5::getDatadouble() const
{
   try
   {
      Exception::dontPrint();
      H5std_string FILE_NAME(LoadH5::filename);
      H5File file(FILE_NAME, H5F_ACC_RDONLY);
      DataSet dataset = file.openDataSet(LoadH5::variable);
      DataType datatype = dataset.getDataType();
      DataSpace dataspace = dataset.getSpace();
      H5T_class_t classt = datatype.getClass();
      if ( classt != 1 )
      {
         std::cout << LoadH5::variable << " is not a float... you can't save this as a float." << std::endl;
         exit(1);
      }
      FloatType ftype = dataset.getFloatType();
      H5std_string order_string;
      H5T_order_t order = ftype.getOrder( order_string);
      size_t size = ftype.getSize();
      double *data = new double;
      if ( order==0 && size == 4 )
      {
         std::cout << "NOTE: This is actually float data. We are casting to double" << std:: endl;
         dataset.read((double*)data, PredType::IEEE_F32LE); // Our standard integer
      }
      else if ( order == 0 && size == 8 ) 
         dataset.read(data, PredType::IEEE_F64LE);
      else if ( order == 1 && size == 4 )
      {
         std::cout << "NOTE: This is actually float data. We are casting to double" << std:: endl;
         dataset.read((double*)data, PredType::IEEE_F32BE);
      }
      else if ( order ==1 && size == 8 )
         dataset.read((double*)data, PredType::IEEE_F64BE);
      else 
         std::cout << "Did not find data type" << std::endl;
      float v = *data;
      delete data;
      dataspace.close();
      datatype.close();
      dataset.close();
      file.close();
      return v;
   }
   catch (FileIException error)
   {
      error.printError();
      double err = -1.;
      return err;
   }
   catch (GroupIException error)
   {
      error.printError();
      double err = -1.;
      return err;
   }
}
// Our int loading function
std::vector<int> LoadH5::getDataVint() const
{
   try
   {
      Exception::dontPrint();
      //std::cout << "Getting int data" << std::endl;
      H5std_string FILE_NAME(LoadH5::filename);
      H5File file(FILE_NAME, H5F_ACC_RDONLY); // Only reads
      DataSet dataset = file.openDataSet(LoadH5::variable);
      DataType datatype = dataset.getDataType();
      DataSpace dataspace = dataset.getSpace();
      const int npts = dataspace.getSimpleExtentNpoints(); // Gets length of data
      H5T_class_t classt = datatype.getClass(); // Gets the data type of the data
      // Let's make a quick error check
      if ( classt != 0 )
      {
         std::cout << LoadH5::variable << " is not an int... you can't save this as an int." << std::endl;
         exit(1);
      }
      int *data = new int[npts]; // allocate at run time what the size will be
      IntType itype = dataset.getIntType();
      H5std_string order_string;
      H5T_order_t order = itype.getOrder( order_string );
      size_t size = itype.getSize();
      if ( order == 0 && size == 1 )
         dataset.read(data, PredType::STD_I8LE); // Our standard integer
      else if ( order== 0 && size == 2 )
         dataset.read(data, PredType::STD_I16LE); // Our standard integer
      else if ( order== 0 && size == 4 )
         dataset.read(data, PredType::STD_I32LE); // Our standard integer
      else if ( order== 0 && size == 8 ) 
         dataset.read(data, PredType::STD_I64LE);
      else if ( order== 1 && size == 1 )
         dataset.read(data, PredType::STD_I8BE); // Our standard integer
      else if ( order== 1 && size == 2 )
         dataset.read(data, PredType::STD_I16BE); // Our standard integer
      else if ( order== 1 && size == 4 )
         dataset.read(data, PredType::STD_I32BE);
      else if ( order== 1 && size == 8 )
         dataset.read(data, PredType::STD_I64BE);
      else 
         std::cout << "Did not find data type" << std::endl;
      std::vector<int> v(data, data + npts); // Arrays are nice, but vectors are better
      // Manage our memory properly
      delete[] data;
      dataspace.close();
      datatype.close();
      dataset.close();
      file.close();
      return v;
   }
   catch (FileIException error)
   {
      error.printError();
      std::vector<int> err{1,-1};
      return err;
   }
   catch (GroupIException error)
   {
      error.printError();
      std::vector<int> err{1,-1};
      return err;
   }
}

// Same as our int function, but with float. Uses IEEE_F32BE
std::vector<float> LoadH5::getDataVfloat() const
{
   try
   {
      Exception::dontPrint();
      //std::cout << "Getting float data" << std::endl;
      H5std_string FILE_NAME(LoadH5::filename);
      H5File file(FILE_NAME, H5F_ACC_RDONLY);
      DataSet dataset = file.openDataSet(LoadH5::variable);
      DataType datatype = dataset.getDataType();
      DataSpace dataspace = dataset.getSpace();
      const int npts = dataspace.getSimpleExtentNpoints();
      H5T_class_t classt = datatype.getClass();
      if ( classt != 1 )
      {
         std::cout << LoadH5::variable << " is not a float... you can't save this as a float." << std::endl;
         exit(1);
      }
      FloatType ftype = dataset.getFloatType();
      H5std_string order_string;
      H5T_order_t order = ftype.getOrder( order_string);
      size_t size = ftype.getSize();
      float *data = new float[npts];
      if ( order == 0 && size == 4 )
         dataset.read(data, PredType::IEEE_F32LE); // Our standard integer
      else if ( order == 0 && size == 8 ) 
      {
         dataset.read((float*)data, PredType::IEEE_F64LE);
         std::cout << "NOTE: This is actually double data. We are casting to float" << std:: endl;
      }
      else if ( order == 1 && size == 4 )
         dataset.read(data, PredType::IEEE_F32BE);
      else if ( (order_string == "Big endian byte order_stringing (1)" || order == 1) && size == 8 )
      {
         std::cout << "NOTE: This is actually double data. We are casting to float" << std:: endl;
         dataset.read((float*)data, PredType::IEEE_F64BE);
      }
      else 
         std::cout << "Did not find data type" << std::endl;
      std::vector<float> v(data, data + npts);
      delete[] data;
      dataspace.close();
      datatype.close();
      dataset.close();
      file.close();
      return v;
   }
   catch (FileIException error)
   {
      error.printError();
      std::vector<float> err{1,-1.};
      return err;
   }
   catch (GroupIException error)
   {
      error.printError();
      std::vector<float> err{1,-1.};
      return err;
   }
}

// Same as our int function, but with double
std::vector<double> LoadH5::getDataVDouble() const
{
   try
   {
      Exception::dontPrint();
      H5std_string FILE_NAME(LoadH5::filename);
      H5File file(FILE_NAME, H5F_ACC_RDONLY);
      DataSet dataset = file.openDataSet(LoadH5::variable);
      DataType datatype = dataset.getDataType();
      DataSpace dataspace = dataset.getSpace();
      const int npts = dataspace.getSimpleExtentNpoints();
      H5T_class_t classt = datatype.getClass();
      if ( classt != 1 )
      {
         std::cout << LoadH5::variable << " is not a float... you can't save this as a float." << std::endl;
         exit(1);
      }
      FloatType ftype = dataset.getFloatType();
      H5std_string order_string;
      H5T_order_t order = ftype.getOrder( order_string);
      size_t size = ftype.getSize();
      double *data = new double[npts];
      if ( order==0 && size == 4 )
      {
         std::cout << "NOTE: This is actually float data. We are casting to double" << std:: endl;
         dataset.read((double*)data, PredType::IEEE_F32LE); // Our standard integer
      }
      else if ( order == 0 && size == 8 ) 
         dataset.read(data, PredType::IEEE_F64LE);
      else if ( order == 1 && size == 4 )
      {
         std::cout << "NOTE: This is actually float data. We are casting to double" << std:: endl;
         dataset.read((double*)data, PredType::IEEE_F32BE);
      }
      else if ( order ==1 && size == 8 )
         dataset.read((double*)data, PredType::IEEE_F64BE);
      else 
         std::cout << "Did not find data type" << std::endl;
      std::vector<double> v(data, data + npts);
      delete[] data;
      dataspace.close();
      datatype.close();
      dataset.close();
      file.close();
      return v;
   }
   catch (FileIException error)
   {
      error.printError();
      std::vector<double> err{1,-1.};
      return err;
   }
   catch (GroupIException error)
   {
      error.printError();
      std::vector<double> err{1,-1.};
      return err;
   }
}

//
// 2-D VECTORS
//
// Same as our int function, but with double
// INT
std::vector<std::vector<int> > LoadH5::getData2Dint() const
{
   try
   {
      Exception::dontPrint();
      H5std_string FILE_NAME(LoadH5::filename);
      H5File file(FILE_NAME, H5F_ACC_RDONLY);
      DataSet dataset = file.openDataSet(LoadH5::variable);
      DataType datatype = dataset.getDataType();
      DataSpace dataspace = dataset.getSpace();
      int rank = dataspace.getSimpleExtentNdims();
      hsize_t dims[rank];
      dataspace.getSimpleExtentDims(dims);
      FloatType ftype = dataset.getFloatType();
      H5std_string order_string;
      H5T_order_t order = ftype.getOrder( order_string);
      size_t size = ftype.getSize();
      size_t dim1 = dims[0];
      size_t dim2 = dims[1];
      auto data = new int[dim1*dim2];
      auto md = new int*[dim1];
      for ( size_t i = 0; i < dim1; ++i )
         md[i] = data + i*dim2;

      if ( order==0 && size == 1 )
         dataset.read(data, PredType::STD_I8LE); // Our standard integer
      else if ( order == 1 && size == 1 ) 
         dataset.read(data, PredType::STD_I8BE);
      else if ( order == 0 && size == 2 ) 
         dataset.read(data, PredType::STD_I16LE);
      else if ( order == 1 && size == 2 ) 
         dataset.read(data, PredType::STD_I16BE);
      else if ( order == 0 && size == 4 ) 
         dataset.read(data, PredType::STD_I32LE);
      else if ( order == 1 && size == 4 ) 
         dataset.read(data, PredType::STD_I32BE);
      else if ( order == 0 && size == 8 ) 
         dataset.read(data, PredType::STD_I64LE);
      else if ( order == 1 && size == 8 ) 
         dataset.read(data, PredType::STD_I64BE);
      else 
         std::cout << "Did not find data type" << std::endl;
      std::vector<std::vector<int> > v(dim1, std::vector<int>(dim2,0));//data, data + npts);
      // Assign 2D vector
      for ( size_t i = 0; i < dim1; ++i )
         for ( size_t j = 0; j < dim2; ++j )
            v[i][j] = md[i][j];
      delete[] md;
      delete data;
      dataspace.close();
      datatype.close();
      dataset.close();
      file.close();
      return v;
   }
   catch (FileIException error)
   {
      error.printError();
      std::vector<std::vector<int> > err{1,std::vector<int>(1,-1)};
      return err;
   }
   catch (GroupIException error)
   {
      error.printError();
      std::vector<std::vector<int> > err{1,std::vector<int>(1,-1)};
      return err;
   }
}
// FLOAT
std::vector<std::vector<float> > LoadH5::getData2Dfloat() const
{
   try
   {
      Exception::dontPrint();
      H5std_string FILE_NAME(LoadH5::filename);
      H5File file(FILE_NAME, H5F_ACC_RDONLY);
      DataSet dataset = file.openDataSet(LoadH5::variable);
      DataType datatype = dataset.getDataType();
      DataSpace dataspace = dataset.getSpace();
      int rank = dataspace.getSimpleExtentNdims();
      hsize_t dims[rank];
      dataspace.getSimpleExtentDims(dims);
      FloatType ftype = dataset.getFloatType();
      H5std_string order_string;
      H5T_order_t order = ftype.getOrder( order_string);
      size_t size = ftype.getSize();
      size_t dim1 = dims[0];
      size_t dim2 = dims[1];
      //float data[dim1][dim2];
      auto data = new float[dim1*dim2];
      auto md   = new float*[dim1];
      for ( size_t i = 0; i < dim1; ++i )
         md[i] = data + i*dim2;
      if ( order==0 && size == 4 )
         dataset.read(data, PredType::IEEE_F32LE); // Our standard integer
      else if ( order == 0 && size == 8 ) 
         dataset.read(data, PredType::IEEE_F64LE);
      else if ( order == 1  && size == 4 )
         dataset.read(data, PredType::IEEE_F32BE);
      else if ( order ==1 && size == 8 )
         dataset.read(data, PredType::IEEE_F64BE);
      else 
         std::cout << "Did not find data type" << std::endl;
      std::vector<std::vector<float> > v(dim1, std::vector<float>(dim2,0));//data, data + npts);
      // Assign 2D vector
      for ( size_t i = 0; i < dim1; ++i )
         for ( size_t j = 0; j < dim2; ++j )
            v[i][j] = md[i][j];
      delete[] md;
      delete data;
      dataspace.close();
      datatype.close();
      dataset.close();
      file.close();
      return v;
   }
   catch (FileIException error)
   {
      error.printError();
      std::vector<std::vector<float> > err{1,std::vector<float>(1,-1.)};
      return err;
   }
   catch (GroupIException error)
   {
      error.printError();
      std::vector<std::vector<float> > err{1,std::vector<float>(1,-1.)};
      return err;
   }
}

// DOUBLE
std::vector<std::vector<double> > LoadH5::getData2Ddouble() const
{
   try
   {
      Exception::dontPrint();
      H5std_string FILE_NAME(LoadH5::filename);
      H5File file(FILE_NAME, H5F_ACC_RDONLY);
      DataSet dataset = file.openDataSet(LoadH5::variable);
      DataType datatype = dataset.getDataType();
      DataSpace dataspace = dataset.getSpace();
      int rank = dataspace.getSimpleExtentNdims();
      hsize_t dims[rank];
      dataspace.getSimpleExtentDims(dims);
      FloatType ftype = dataset.getFloatType();
      H5std_string order_string;
      H5T_order_t order = ftype.getOrder( order_string);
      size_t size = ftype.getSize();
      size_t dim1 = dims[0];
      size_t dim2 = dims[1];
      //double data[dim1][dim2];
      auto data = new double[dim1*dim2];
      auto md   = new double*[dim1];
      for ( size_t i = 0; i < dim1; ++i )
         md[i] = data + i*dim2;
      if ( order==0 && size == 4 )
         dataset.read(data, PredType::IEEE_F32LE); // Our standard integer
      else if ( order == 0 && size == 8 ) 
         dataset.read(data, PredType::IEEE_F64LE);
      else if ( order == 1 && size == 4 )
         dataset.read(data, PredType::IEEE_F32BE);
      else if ( order ==1 && size == 8 )
         dataset.read(data, PredType::IEEE_F64BE);
      else 
         std::cout << "Did not find data type" << std::endl;
      std::vector<std::vector<double> > v(dim1, std::vector<double>(dim2,0));//data, data + npts);
      // Assign 2D vector
      for ( size_t i = 0; i < dim1; ++i )
         for ( size_t j = 0; j < dim2; ++j )
            v[i][j] = md[i][j];
      delete[] md;
      delete data;
      dataspace.close();
      datatype.close();
      dataset.close();
      file.close();
      return v;
   }
   catch (FileIException error)
   {
      error.printError();
      std::vector<std::vector<double> > err{1,std::vector<double>(1,-1.)};
      return err;
   }
   catch (GroupIException error)
   {
      error.printError();
      std::vector<std::vector<double> > err{1,std::vector<double>(1,-1.)};
      return err;
   }
}

#endif
