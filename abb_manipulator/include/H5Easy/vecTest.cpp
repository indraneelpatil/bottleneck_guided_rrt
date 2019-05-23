#include <iostream>
#include <vector>
#include "H5Easy.h"
#include "H5Cpp.h"

using namespace std;
using namespace H5;

#define dim1 5
#define dim2 3
int main()
{
   vector<vector<double> > v(dim1, vector<double>(dim2));
   for ( size_t i = 0; i < dim1; ++i )
      for ( size_t j = 0; j < dim2; ++j )
         v[i][j] = 3;
   vector<int> vi{0,1,2,3};
   //for ( size_t i = 0; i < dim1; ++i )
   //{
   //   for ( size_t j = 0; j < dim2; ++j )
   //      cout << v[i][j] << " ";
   //   cout << endl;
   //}
   cout << "v is shape: (" << v.size() << "," << v[0].size() <<")" << endl;
   WriteH5 dta;
   dta.setFileName("dim.h5");
   dta.setVarName("vecvecdouble");
   dta.writeData(v);
   dta.setVarName("vecInt");
   dta.writeData(vi);
   cout << "Done writing!\tStarting read" << endl;
   
   LoadH5 data;
   data.setFileName("dim.h5");
   data.setVarName("vecvecdouble");
   vector<vector<double> > vv = data.getData();
   cout << "Data is read" << endl;
   for ( size_t i = 0; i < dim1; ++i )
   {
      for ( size_t j = 0; j < dim2; ++j )
         cout << vv[i][j] << "\t";
      cout << endl;
   }

   return 0;
}
