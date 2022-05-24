// Translation of ImageTracer.java into C++

/*
  ImageTracer.cpp
  (Desktop version in C++. See ImageTracerAndroid.java for the Android version.)
  Simple raster image tracer and vectorizer written in C++.
  This is a port of https://github.com/jankovicsandras/imagetracerjava

  by Dov Grobgeld <dov.grobgeld@gmail.com>
 */

/*

The Unlicense / PUBLIC DOMAIN

This is free and unencumbered software released into the public domain.

Anyone is free to copy, modify, publish, use, compile, sell, or
distribute this software, either in source code form or as a compiled
binary, for any purpose, commercial or non-commercial, and by any
means.

In jurisdictions that recognize copyright laws, the author or authors
of this software dedicate any and all copyright interest in the
software to the public domain. We make this dedication for the benefit
of the public at large and to the detriment of our heirs and
successors. We intend this dedication to be an overt act of
relinquishment in perpetuity of all present and future rights to this
software under copyright law.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.

For more information, please refer to http://unlicense.org/

 */

#include <vector>
#include <string>
#include <array>
#include <map>
#include <fstream>
#include <sstream>
#include <fmt/core.h>
#include <cmath>


using namespace std;
using namespace fmt;

const string versionnumber = "1.1.2";

// Forward declarations
class ImageData;
class IndexedImage;
static map<string,double> checkoptions (map<string,double> options);
static string imagedataToSVG (const ImageData& imgd,
                              map<string,double> options,
                              const vector<array<uint8_t,4>>& palette);
static IndexedImage imagedataToTracedata (const ImageData& imgd,
                                          const map<string,double>& options,
                                          const vector<array<uint8_t,4>>& palette);
static string getsvgstring (IndexedImage ii, map<string,double> options);
static IndexedImage colorquantization (ImageData imgd,
                                       vector<array<uint8_t,4>> palette,
                                       const map<string,double>& options);
static vector<vector<vector<int>>> layering (IndexedImage ii);
static vector<vector<vector<int>>> pathscan (vector<vector<int>>& arr,
                                             int pathomit);
static vector<vector<vector<vector<int>>>> batchpathscan(
  const vector<vector<vector<int>>>& layers, int pathomit);
static vector<vector<vector<double>>> internodes (vector<vector<vector<int>>> paths);
static vector<vector<vector<vector<double>>>> batchinternodes (vector<vector<vector<vector<int>>>> bpaths);
static vector<vector<double>> tracepath (vector<vector<double>> path,
                                         double ltreshold,
                                         double qtreshold);
static vector<vector<double>> fitseq (vector<vector<double>> path,
                                      double ltreshold,
                                      double qtreshold,
                                      int seqstart,
                                      int seqend);
static vector<vector<vector<double>>> batchtracepaths (vector<vector<vector<double>>> internodepaths, double ltres,double qtres);
static vector<vector<vector<vector<double>>>> batchtracelayers (vector<vector<vector<vector<double>>>> binternodes, double ltres, double qtres);
static vector<array<uint8_t,4>> samplepalette (int numberofcolors,
                                               ImageData imgd);
static vector<array<uint8_t,4>> generatepalette (int numberofcolors);
static ImageData blur (ImageData imgd, double rad, double del);
static string tosvgcolorstr (const array<uint8_t,4>& c);

// Container for the color-indexed image before and tracedata after vectorizing
class IndexedImage
{
  public:
  int width, height;
  vector<vector<int>> aray; // aray[x][y] of palette colors
  vector<array<uint8_t,4>> palette;// array[palettelength][4] RGBA color palette
  vector<vector<vector<vector<double>>>> layers; // tracedata

  public:
  // Constructor
  IndexedImage(const vector<vector<int>>& marray,
               const vector<array<uint8_t,4>>& mpalette)
  {
    aray = marray;
    palette = mpalette;
    // Color quantization adds +2 to the original width and height
    width = marray[0].size()-2;
    height = marray.size()-2;
  }
};

class ImageData
{
  public:
  int width, height;
  vector<uint8_t> data; // raw byte data: R G B A R G B A ...
                        
  public:
  ImageData(int mwidth=0, int mheight=0, const vector<uint8_t>& mdata=vector<uint8_t>(0,0))
  {
    width = mwidth; height = mheight; data = mdata;
  }
};

// Loading a file to ImageData, ARGB byte order. Currently only
// 8-bit PPM images are supported
static ImageData loadImageData (const string& filename) 
{
  // Currently read a ppm file this should be updated
  fstream file(filename);

  if (!file.good())
    throw runtime_error("Failed loading image!");

  string Line;
  getline(file, Line);
  if (Line!="P6")
    throw runtime_error("Expected a P6 (PPM) file!");

  // Skip comments
  getline(file,Line);
  while(Line.size() && Line[0]=='#')
    getline(file,Line);

  // Get image size
  stringstream ss(Line);
  int Width, Height;
  ss >> Width >> Height;
    
  // Get bit depth. We only support 8-bit images
  getline(file,Line);
  if (Line!="255")
    throw runtime_error("Expected a 8-bit ppm file");

  // Allocate and read the image data one pixel at a time
  vector<uint8_t> Data(Width*Height*4, 0);
  for (int i=0; i<Width * Height; i++)
  {
    uint32_t pixel;
    file.read((char*)&pixel, 3);
    // Result should be RGBA order
    Data[4*i+0] = pixel & 0xff;         
    Data[4*i+1] = (pixel>>8) & 0xff;
    Data[4*i+2] = (pixel>>16) & 0xff;
    Data[4*i+3] = 0xff;  // alpha
  }

  return ImageData(Width,Height,Data);
}

////////////////////////////////////////////////////////////
//
//  User friendly functions
//
////////////////////////////////////////////////////////////

// Loading an image from a file, tracing when loaded, then returning the SVG string
static string imageToSVG (
  const string& filename,
  map<string,double> options,
  const vector<array<uint8_t,4>>& palette = vector<array<uint8_t,4>>())
{
  options = checkoptions(options);
  ImageData imgd = loadImageData(filename);
  return imagedataToSVG(imgd,options,palette);
}// End of imageToSVG()

// Tracing ImageData, then returning the SVG string
static string imagedataToSVG (const ImageData& imgd,
                              map<string,double> options,
                              const vector<array<uint8_t,4>>& palette)
{
  options = checkoptions(options);
  IndexedImage ii = imagedataToTracedata(imgd,options,palette);
  return getsvgstring(ii, options);
}// End of imagedataToSVG()


// Loading an image from a file, tracing when loaded, then returning IndexedImage with tracedata in layers
static IndexedImage imageToTracedata (const string& filename,
                                      map<string,double> options,
                                      const vector<array<uint8_t,4>>& palette)
{
  options = checkoptions(options);
  ImageData imgd = loadImageData(filename);
  return imagedataToTracedata(imgd,options,palette);
}// End of imageToTracedata()

// Tracing ImageData, then returning IndexedImage with tracedata in layers
static IndexedImage imagedataToTracedata (const ImageData& imgd,
                                          const map<string,double>& options,
                                          const vector<array<uint8_t,4>>& palette)
{
  // 1. Color quantization
  IndexedImage ii = colorquantization(imgd, palette, options);
  // 2. Layer separation and edge detection
  vector<vector<vector<int>>> rawlayers = layering(ii);
  // 3. Batch pathscan
  vector<vector<vector<vector<int>>>> bps = batchpathscan(
    rawlayers,
    (int)(floor(options.at("pathomit"))));
  // 4. Batch interpollation
  vector<vector<vector<vector<double>>>> bis = batchinternodes(bps);
  // 5. Batch tracing
  ii.layers = batchtracelayers(bis,options.at("ltres"),options.at("qtres"));
  return ii;
}// End of imagedataToTracedata()


// creating options object, setting defaults for missing values
static map<string,double> checkoptions (map<string,double> options)
{
  // Tracing
  if(!options.count("ltres")){ options["ltres"]=1; }
  if(!options.count("qtres")){ options["qtres"]=1; }
  if(!options.count("pathomit")){ options["pathomit"]=8; }
  // Color quantization
  if(!options.count("colorsampling")){ options["colorsampling"]=1; }
  if(!options.count("numberofcolors")){ options["numberofcolors"]=16; }
  if(!options.count("mincolorratio")){ options["mincolorratio"]=2; }
  if(!options.count("colorquantcycles")){ options["colorquantcycles"]=3; }
  // SVG rendering
  if(!options.count("scale")){ options["scale"]=1; }
  if(!options.count("simplifytolerance")){ options["simplifytolerance"]=0; }
  if(!options.count("roundcoords")){ options["roundcoords"]=1; }
  if(!options.count("lcpr")){ options["lcpr"]=0; }
  if(!options.count("qcpr")){ options["qcpr"]=0; }
  if(!options.count("desc")){ options["desc"]=1; }
  if(!options.count("viewbox")){ options["viewbox"]=0; }
  // Blur
  if(!options.count("blurradius")){ options["blurradius"]=0; }
  if(!options.count("blurdelta")){ options["blurdelta"]=20; }

  return options;
}// End of checkoptions()

static double frand()
{
    return (double)rand() / RAND_MAX;
}

////////////////////////////////////////////////////////////
//
//  Vectorizing functions
//
////////////////////////////////////////////////////////////

// 1. Color quantization repeated "cycles" times, based on K-means clustering
// https://en.wikipedia.org/wiki/Color_quantization    https://en.wikipedia.org/wiki/K-means_clustering
static IndexedImage colorquantization (ImageData imgd, vector<array<uint8_t,4>> palette, const map<string,double>& options)
{
  int numberofcolors = (int)floor(options.at("numberofcolors"));
  double minratio = options.at("mincolorratio");
  int cycles = (int)floor(options.at("colorquantcycles"));

  // Creating indexed color array arr which has a boundary filled with -1 in every direction
  vector<vector<int>> arr(imgd.height+2,
                          vector<int>(imgd.width+2));

  for(int j=0; j<(imgd.height+2); j++)
  {
    arr[j][0] = -1;
    arr[j][imgd.width+1 ] = -1;
  }

  for(int i=0; i<(imgd.width+2) ; i++)
  {
    arr[0][i] = -1;
    arr[imgd.height+1][i] = -1;
  }

  int idx=0, cd,cdl,ci,c1,c2,c3,c4;

  // Use custom palette if pal is defined or sample or generate custom length palette
  if(palette.size()==0)
  {
    if(options.at("colorsampling")!=0)
      palette = samplepalette(numberofcolors,imgd);
    else
      palette = generatepalette(numberofcolors);
  }

  // Selective Gaussian blur preprocessing
  if( options.at("blurradius") > 0 )
  {
    imgd = blur( imgd, options.at("blurradius"), options.at("blurdelta") );
  }

  vector<array<int,5>> paletteacc(palette.size(), array<int,5>());

  // Repeat clustering step "cycles" times
  for(int cnt=0;cnt<cycles;cnt++)
  {
    // Average colors from the second iteration
    if(cnt>0){
      // averaging paletteacc for palette
      double ratio;
      for(size_t k=0;k<palette.size();k++)
      {
        // averaging
        if(paletteacc[k][3]>0)
        {
          palette[k][0] = (uint8_t)(paletteacc[k][0] / paletteacc[k][4]);
          palette[k][1] = (uint8_t)(paletteacc[k][1] / paletteacc[k][4]);
          palette[k][2] = (uint8_t)(paletteacc[k][2] / paletteacc[k][4]);
          palette[k][3] = (uint8_t)(paletteacc[k][3] / paletteacc[k][4]);
        }
        ratio = ( (double)(paletteacc[k][4]) / (double)(imgd.width*imgd.height) );

        // Randomizing a color, if there are too few pixels and there will be a new cycle
        if( (ratio<minratio) && (cnt<(cycles-1)) )
        {
          palette[k][0] = (uint8_t)floor(frand()*255);
          palette[k][1] = (uint8_t)floor(frand()*255);
          palette[k][2] = (uint8_t)floor(frand()*255);
          palette[k][3] = (uint8_t)floor(frand()*255);
        }

      }// End of palette loop
    }// End of Average colors from the second iteration

    // Reseting palette accumulator for averaging
    for(size_t i=0;i<palette.size();i++)
    {
      paletteacc[i][0]=0;
      paletteacc[i][1]=0;
      paletteacc[i][2]=0;
      paletteacc[i][3]=0;
      paletteacc[i][4]=0;
    }

    // loop through all pixels
    for(int j=0;j<imgd.height;j++)
    {
      for(int i=0;i<imgd.width;i++)
      {

        idx = ((j*imgd.width)+i)*4;

        // find closest color from palette by measuring (rectilinear) color distance between this pixel and all palette colors
        cdl = 256+256+256+256; ci=0;
        for(size_t k=0;k<palette.size();k++)
        {

          // In my experience, https://en.wikipedia.org/wiki/Rectilinear_distance works better than https://en.wikipedia.org/wiki/Euclidean_distance
          c1 = abs(palette[k][0]-imgd.data[idx]);
          c2 = abs(palette[k][1]-imgd.data[idx+1]);
          c3 = abs(palette[k][2]-imgd.data[idx+2]);
          c4 = abs(palette[k][3]-imgd.data[idx+3]);
          cd = c1+c2+c3+(c4*4); // weighted alpha seems to help images with transparency

          // Remember this color if this is the closest yet
          if(cd<cdl)
          {
            cdl = cd;
            ci = k;
          }

        }// End of palette loop

        // add to palettacc
        paletteacc[ci][0] += imgd.data[idx];
        paletteacc[ci][1] += imgd.data[idx+1];
        paletteacc[ci][2] += imgd.data[idx+2];
        paletteacc[ci][3] += imgd.data[idx+3];
        paletteacc[ci][4]++;

        arr[j+1][i+1] = ci;
      }// End of i loop
    }// End of j loop

  }// End of Repeat clustering step "cycles" times

  return IndexedImage(arr, palette);
}// End of colorquantization


// Generating a palette with numberofcolors, array[numberofcolors][4] where [i][0] = R ; [i][1] = G ; [i][2] = B ; [i][3] = A
static vector<array<uint8_t,4>> generatepalette (int numberofcolors)
{
  vector<array<uint8_t,4>> palette(numberofcolors, array<uint8_t,4>());

  if (numberofcolors<8)
  {
    // Grayscale
    double graystep = 255.0/(double)(numberofcolors-1);
    for(uint8_t ccnt=0;ccnt<numberofcolors;ccnt++)
    {
      palette[ccnt][0] = (uint8_t)(round(ccnt*graystep));
      palette[ccnt][1] = (uint8_t)(round(ccnt*graystep));
      palette[ccnt][2] = (uint8_t)(round(ccnt*graystep));
      palette[ccnt][3] = (uint8_t)255;
    }
  }
  else
  {

    // RGB color cube
    int colorqnum = (int) floor(pow(numberofcolors, 1.0/3.0)); // Number of points on each edge on the RGB color cube
    int colorstep = (int) floor(255/(colorqnum-1)); // distance between points
    int ccnt = 0;
    for(int rcnt=0;rcnt<colorqnum;rcnt++)
    {
      for(int gcnt=0;gcnt<colorqnum;gcnt++)
      {
        for(int bcnt=0;bcnt<colorqnum;bcnt++)
        {
          palette[ccnt][0] = (uint8_t)(rcnt*colorstep);
          palette[ccnt][1] = (uint8_t)(gcnt*colorstep);
          palette[ccnt][2] = (uint8_t)(bcnt*colorstep);
          palette[ccnt][3] = (uint8_t)255;
          ccnt++;
        }// End of blue loop
      }// End of green loop
    }// End of red loop

    // Rest is random
    for(int rcnt=ccnt;rcnt<numberofcolors;rcnt++)
    {
      palette[ccnt][0] = (uint8_t)(floor(frand()*255));
      palette[ccnt][1] = (uint8_t)(floor(frand()*255));
      palette[ccnt][2] = (uint8_t)(floor(frand()*255));
      palette[ccnt][3] = (uint8_t)(floor(frand()*255));
    }

  }// End of numberofcolors check

  return palette;
};// End of generatepalette()


static vector<array<uint8_t,4>> samplepalette (int numberofcolors,
                                               ImageData imgd)
{
  int idx=0;
  vector<array<uint8_t,4>> palette(numberofcolors, array<uint8_t,4>());

  for(int i=0; i<numberofcolors; i++)
  {
    idx = (int) (floor( (frand() * imgd.data.size()) / 4 ) * 4);
    palette[i][0] = imgd.data[idx  ];
    palette[i][1] = imgd.data[idx+1];
    palette[i][2] = imgd.data[idx+2];
    palette[i][3] = imgd.data[idx+3];
  }
  return palette;
}// End of samplepalette()


// 2. Layer separation and edge detection
// Edge node types ( ▓:light or 1; ░:dark or 0 )
// 12  ░░  ▓░  ░▓  ▓▓  ░░  ▓░  ░▓  ▓▓  ░░  ▓░  ░▓  ▓▓  ░░  ▓░  ░▓  ▓▓
// 48  ░░  ░░  ░░  ░░  ░▓  ░▓  ░▓  ░▓  ▓░  ▓░  ▓░  ▓░  ▓▓  ▓▓  ▓▓  ▓▓
//     0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15
//
static vector<vector<vector<int>>> layering (IndexedImage ii)
{
  // Creating layers for each indexed color in arr
  int val=0, aw = ii.aray[0].size(), ah = ii.aray.size(), n1,n2,n3,n4,n5,n6,n7,n8;
  
  vector<vector<vector<int>>> layers(
    ii.palette.size(),
    vector<vector<int>>(ah, vector<int>(aw,0)));

  // Looping through all pixels and calculating edge node type
  for(int j=1; j<(ah-1); j++)
  {
    for(int i=1; i<(aw-1); i++)
    {
      // This pixel's indexed color
      val = ii.aray[j][i];

      // Are neighbor pixel colors the same?
      n1 = ii.aray[j-1][i-1]==val ? 1 : 0;
      n2 = ii.aray[j-1][i  ]==val ? 1 : 0;
      n3 = ii.aray[j-1][i+1]==val ? 1 : 0;
      n4 = ii.aray[j  ][i-1]==val ? 1 : 0;
      n5 = ii.aray[j  ][i+1]==val ? 1 : 0;
      n6 = ii.aray[j+1][i-1]==val ? 1 : 0;
      n7 = ii.aray[j+1][i  ]==val ? 1 : 0;
      n8 = ii.aray[j+1][i+1]==val ? 1 : 0;

      // this pixel"s type and looking back on previous pixels
      layers[val][j+1][i+1] = 1 + (n5 * 2) + (n8 * 4) + (n7 * 8) ;
      if(n4==0)
        layers[val][j+1][i  ] = 0 + 2 + (n7 * 4) + (n6 * 8) ; 
      if(n2==0)
        layers[val][j  ][i+1] = 0 + (n3*2) + (n5 * 4) + 8 ; 
      if(n1==0)
        layers[val][j  ][i  ] = 0 + (n2*2) + 4 + (n4 * 8) ; 

    }// End of i loop
  }// End of j loop

  return layers;
}// End of layering()


// Lookup tables for pathscan
static constexpr uint8_t pathscan_dir_lookup[] = {0,0,3,0, 1,0,3,0, 0,3,3,1, 0,3,0,0};
static constexpr bool pathscan_holepath_lookup[] = {false,false,false,false, false,false,false,true, false,false,false,true, false,true,true,false };
// pathscan_combined_lookup[ arr[py][px] ][ dir ] = [nextarrpypx, nextdir, deltapx, deltapy];
static constexpr int8_t pathscan_combined_lookup[16][4][4] = {
    {{-1,-1,-1,-1}, {-1,-1,-1,-1}, {-1,-1,-1,-1}, {-1,-1,-1,-1}},// arr[py][px]==0 is invalid
    {{ 0, 1, 0,-1}, {-1,-1,-1,-1}, {-1,-1,-1,-1}, { 0, 2,-1, 0}},
    {{-1,-1,-1,-1}, {-1,-1,-1,-1}, { 0, 1, 0,-1}, { 0, 0, 1, 0}},
    {{ 0, 0, 1, 0}, {-1,-1,-1,-1}, { 0, 2,-1, 0}, {-1,-1,-1,-1}},

    {{-1,-1,-1,-1}, { 0, 0, 1, 0}, { 0, 3, 0, 1}, {-1,-1,-1,-1}},
    {{13, 3, 0, 1}, {13, 2,-1, 0}, { 7, 1, 0,-1}, { 7, 0, 1, 0}},
    {{-1,-1,-1,-1}, { 0, 1, 0,-1}, {-1,-1,-1,-1}, { 0, 3, 0, 1}},
    {{ 0, 3, 0, 1}, { 0, 2,-1, 0}, {-1,-1,-1,-1}, {-1,-1,-1,-1}},

    {{ 0, 3, 0, 1}, { 0, 2,-1, 0}, {-1,-1,-1,-1}, {-1,-1,-1,-1}},
    {{-1,-1,-1,-1}, { 0, 1, 0,-1}, {-1,-1,-1,-1}, { 0, 3, 0, 1}},
    {{11, 1, 0,-1}, {14, 0, 1, 0}, {14, 3, 0, 1}, {11, 2,-1, 0}},
    {{-1,-1,-1,-1}, { 0, 0, 1, 0}, { 0, 3, 0, 1}, {-1,-1,-1,-1}},

    {{ 0, 0, 1, 0}, {-1,-1,-1,-1}, { 0, 2,-1, 0}, {-1,-1,-1,-1}},
    {{-1,-1,-1,-1}, {-1,-1,-1,-1}, { 0, 1, 0,-1}, { 0, 0, 1, 0}},
    {{ 0, 1, 0,-1}, {-1,-1,-1,-1}, {-1,-1,-1,-1}, { 0, 2,-1, 0}},
    {{-1,-1,-1,-1}, {-1,-1,-1,-1}, {-1,-1,-1,-1}, {-1,-1,-1,-1}}// arr[py][px]==15 is invalid
};


// 3. Walking through an edge node array, discarding edge node types 0 and 15 and creating paths from the rest.
// Walk directions (dir): 0 > ; 1 ^ ; 2 < ; 3 v
// Edge node types ( ▓:light or 1; ░:dark or 0 )
// ░░  ▓░  ░▓  ▓▓  ░░  ▓░  ░▓  ▓▓  ░░  ▓░  ░▓  ▓▓  ░░  ▓░  ░▓  ▓▓
// ░░  ░░  ░░  ░░  ░▓  ░▓  ░▓  ░▓  ▓░  ▓░  ▓░  ▓░  ▓▓  ▓▓  ▓▓  ▓▓
// 0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15
//
static vector<vector<vector<int>>> pathscan (vector<vector<int>>& arr,
                                             int pathomit)
{
  vector<vector<vector<int>>> paths;
  int px=0,py=0,w=arr[0].size(),h=arr.size(),dir=0;
  bool pathfinished=true, holepath = false;
  for(int j=0;j<h;j++)
  {
    for(int i=0;i<w;i++)
    {
      if((arr[j][i]!=0)&&(arr[j][i]!=15))
      {
        // Init
        px = i; py = j;
        paths.push_back(vector<vector<int>>());
        auto& thispath = paths.back();
        pathfinished = false;

        // fill paths will be drawn, but hole paths are also required to remove unnecessary edge nodes
        dir = pathscan_dir_lookup[ arr[py][px] ];
        holepath = pathscan_holepath_lookup[ arr[py][px] ];

        // Path points loop
        while(!pathfinished)
        {
          // New path point
          thispath.push_back(vector<int>(3,0));
          thispath.back()[0] = px-1;
          thispath.back()[1] = py-1;
          thispath.back()[2] = arr[py][px];

          // Next: look up the replacement, direction and coordinate changes = clear this cell, turn if required, walk forward
          const int8_t* lookuprow = pathscan_combined_lookup[ arr[py][px] ][ dir ];
          arr[py][px] = lookuprow[0];
          dir = lookuprow[1];
          px += lookuprow[2];
          py += lookuprow[3];

          // Close path
          if(((px-1)==thispath[0][0])&&((py-1)==thispath[0][1]))
          {
            pathfinished = true;
            // Discarding 'hole' type paths and paths shorter than pathomit
            if( (holepath) || ((int)thispath.size()<pathomit) )
            {
              paths.pop_back();
            }
          }

        }// End of Path points loop

      }// End of Follow path

    }// End of i loop
  }// End of j loop

  return paths;
}// End of pathscan()


// 3. Batch pathscan
static vector<vector<vector<vector<int>>>> batchpathscan(
  const vector<vector<vector<int>>>& layers, int pathomit)
{
  vector<vector<vector<vector<int>>>> bpaths;

  for (auto layer : layers) {
    bpaths.push_back(pathscan(layer,pathomit));
  }
  return bpaths;
}


// 4. interpolating between path points for nodes with 8 directions ( East, SouthEast, S, SW, W, NW, N, NE )
static vector<vector<vector<double>>> internodes (vector<vector<vector<int>>> paths)
{
  vector<vector<vector<double>>> ins;
  vector<double> nextpoint(2,0.0);
  vector<int> pp1, pp2, pp3;
  int palen=0,nextidx=0,nextidx2=0;

  // paths loop
  for(size_t pacnt=0; pacnt<paths.size(); pacnt++)
  {
    ins.push_back(vector<vector<double>>());
    auto& thisinp = ins.back();
    palen = paths[pacnt].size();

    // pathpoints loop
    for(int pcnt=0;pcnt<palen;pcnt++)
    {
      // interpolate between two path points
      nextidx = (pcnt+1)%palen; nextidx2 = (pcnt+2)%palen;
      thisinp.push_back(vector<double>(3,0));
      auto& thispoint = thisinp.back();
      pp1 = paths[pacnt][pcnt];
      pp2 = paths[pacnt][nextidx];
      pp3 = paths[pacnt][nextidx2];
      thispoint[0] = (pp1[0]+pp2[0]) / 2.0;
      thispoint[1] = (pp1[1]+pp2[1]) / 2.0;
      nextpoint[0] = (pp2[0]+pp3[0]) / 2.0;
      nextpoint[1] = (pp2[1]+pp3[1]) / 2.0;

      // line segment direction to the next point
      if(thispoint[0] < nextpoint[0])
      {
        if     (thispoint[1] < nextpoint[1])
          thispoint[2] = 1.0; // SouthEast
        else if(thispoint[1] > nextpoint[1])
          thispoint[2] = 7.0; // NE
        else
          thispoint[2] = 0.0; // E
      }
      else if(thispoint[0] > nextpoint[0])
      {
        if     (thispoint[1] < nextpoint[1])
          thispoint[2] = 3.0; // SW
        else if(thispoint[1] > nextpoint[1])
          thispoint[2] = 5.0; // NW
        else
          thispoint[2] = 4.0; // W
      }else{
        if     (thispoint[1] < nextpoint[1])
          thispoint[2] = 2.0; // S
        else if(thispoint[1] > nextpoint[1])
          thispoint[2] = 6.0; // N
        else
          thispoint[2] = 8.0; // center, this should not happen
      }

    }// End of pathpoints loop
  }// End of paths loop
  return ins;
}// End of internodes()


// 4. Batch interpollation
static vector<vector<vector<vector<double>>>> batchinternodes (vector<vector<vector<vector<int>>>> bpaths)
{
  vector<vector<vector<vector<double>>>> binternodes;
  for(size_t k=0; k<bpaths.size(); k++) 
    binternodes.push_back(internodes(bpaths[k]));

  return binternodes;
}


// 5. tracepath() : recursively trying to fit straight and quadratic spline segments on the 8 direction internode path

// 5.1. Find sequences of points with only 2 segment types
// 5.2. Fit a straight line on the sequence
// 5.3. If the straight line fails (an error>ltreshold), find the point with the biggest error
// 5.4. Fit a quadratic spline through errorpoint (project this to get controlpoint), then measure errors on every point in the sequence
// 5.5. If the spline fails (an error>qtreshold), find the point with the biggest error, set splitpoint = (fitting point + errorpoint)/2
// 5.6. Split sequence and recursively apply 5.2. - 5.7. to startpoint-splitpoint and splitpoint-endpoint sequences
// 5.7. TODO? If splitpoint-endpoint is a spline, try to add new points from the next sequence

// This returns an SVG Path segment as a double[7] where
// segment[0] ==1.0 linear  ==2.0 quadratic interpolation
// segment[1] , segment[2] : x1 , y1
// segment[3] , segment[4] : x2 , y2 ; middle point of Q curve, endpoint of L line
// segment[5] , segment[6] : x3 , y3 for Q curve, should be 0.0 , 0.0 for L line
//
// path type is discarded, no check for path.size < 3 , which should not happen

static vector<vector<double>> tracepath (vector<vector<double>> path,
                                         double ltreshold,
                                         double qtreshold)
{
  int pcnt=0, seqend=0; double segtype1, segtype2;
  vector<vector<double>> smp;
  //Double [] thissegment;
  int pathlength = path.size();

  while(pcnt<pathlength)
  {
    // 5.1. Find sequences of points with only 2 segment types
    segtype1 = path[pcnt][2]; segtype2 = -1; seqend=pcnt+1;
    while(
        ((path[seqend][2]==segtype1) || (path[seqend][2]==segtype2) || (segtype2==-1))
        && (seqend<(pathlength-1)))
    {
      if((path[seqend][2]!=segtype1) && (segtype2==-1))
        segtype2 = path[seqend][2];
      seqend++;
    }
    if(seqend==(pathlength-1))
      seqend = 0; 

    // 5.2. - 5.6. Split sequence and recursively apply 5.2. - 5.6. to startpoint-splitpoint and splitpoint-endpoint sequences
    for (auto& v : fitseq(path,ltreshold,qtreshold,pcnt,seqend))
      smp.push_back(v);
    // 5.7. TODO? If splitpoint-endpoint is a spline, try to add new points from the next sequence

    // forward pcnt;
    if(seqend>0)
      pcnt = seqend;
    else
      pcnt = pathlength; 
  }// End of pcnt loop

  return smp;

}// End of tracepath()


// 5.2. - 5.6. recursively fitting a straight or quadratic line segment on this sequence of path nodes,
// called from tracepath()
static vector<vector<double>> fitseq (vector<vector<double>> path,
                                      double ltreshold,
                                      double qtreshold,
                                      int seqstart,
                                      int seqend)
{
  vector<vector<double>> segment;
  int pathlength = path.size();

  // return if invalid seqend
  if((seqend>pathlength)||(seqend<0))
    return segment;

  int errorpoint=seqstart;
  bool curvepass=true;
  double px, py, dist2, errorval=0;
  double tl = (seqend-seqstart); if(tl<0){ tl += pathlength; }
  double vx = (path[seqend][0]-path[seqstart][0]) / tl,
      vy = (path[seqend][1]-path[seqstart][1]) / tl;

  // 5.2. Fit a straight line on the sequence
  int pcnt = (seqstart+1)%pathlength;
  double pl;
  while(pcnt != seqend)
  {
    pl = pcnt-seqstart;
    if(pl<0)
      pl += pathlength; 
    px = path[seqstart][0] + (vx * pl);
    py = path[seqstart][1] + (vy * pl);
    dist2 = ((path[pcnt][0]-px)*(path[pcnt][0]-px)) + ((path[pcnt][1]-py)*(path[pcnt][1]-py));
    if(dist2>ltreshold)
      curvepass=false;
    if(dist2>errorval)
    {
      errorpoint=pcnt;
      errorval=dist2;
    }
    pcnt = (pcnt+1)%pathlength;
  }

  // return straight line if fits
  if(curvepass)
  {
    segment.push_back(vector<double>(7,0));
    auto& thissegment = segment.back();
    thissegment[0] = 1.0;
    thissegment[1] = path[seqstart][0];
    thissegment[2] = path[seqstart][1];
    thissegment[3] = path[seqend][0];
    thissegment[4] = path[seqend][1];
    thissegment[5] = 0.0;
    thissegment[6] = 0.0;
    return segment;
  }

  // 5.3. If the straight line fails (an error>ltreshold), find the point with the biggest error
  int fitpoint = errorpoint; curvepass = true; errorval = 0;

  // 5.4. Fit a quadratic spline through this point, measure errors on every point in the sequence
  // helpers and projecting to get control point
  double t=(fitpoint-seqstart)/tl, t1=(1.0-t)*(1.0-t), t2=2.0*(1.0-t)*t, t3=t*t;
  double cpx = (((t1*path[seqstart][0]) + (t3*path[seqend][0])) - path[fitpoint][0])/-t2 ,
      cpy = (((t1*path[seqstart][1]) + (t3*path[seqend][1])) - path[fitpoint][1])/-t2 ;

  // Check every point
  pcnt = seqstart+1;
  while(pcnt != seqend)
  {
    t=(pcnt-seqstart)/tl; t1=(1.0-t)*(1.0-t); t2=2.0*(1.0-t)*t; t3=t*t;
    px = (t1 * path[seqstart][0]) + (t2 * cpx) + (t3 * path[seqend][0]);
    py = (t1 * path[seqstart][1]) + (t2 * cpy) + (t3 * path[seqend][1]);

    dist2 = ((path[pcnt][0]-px)*(path[pcnt][0]-px)) + ((path[pcnt][1]-py)*(path[pcnt][1]-py));

    if(dist2>qtreshold)
      curvepass=false;
    if(dist2>errorval)
    {
      errorpoint=pcnt; errorval=dist2;
    }
    pcnt = (pcnt+1)%pathlength;
  }

  // return spline if fits
  if(curvepass)
  {
    segment.push_back(vector<double>(7,0));
    auto& thissegment = segment.back();
    thissegment[0] = 2.0;
    thissegment[1] = path[seqstart][0];
    thissegment[2] = path[seqstart][1];
    thissegment[3] = cpx;
    thissegment[4] = cpy;
    thissegment[5] = path[seqend][0];
    thissegment[6] = path[seqend][1];
    return segment;
  }

  // 5.5. If the spline fails (an error>qtreshold), find the point with the biggest error,
  // set splitpoint = (fitting point + errorpoint)/2
  int splitpoint = (fitpoint + errorpoint)/2;

  // 5.6. Split sequence and recursively apply 5.2. - 5.6. to startpoint-splitpoint and splitpoint-endpoint sequences
  segment = fitseq(path,ltreshold,qtreshold,seqstart,splitpoint);
  for (const auto& v : fitseq(path,ltreshold,qtreshold,splitpoint,seqend))
    segment.push_back(v);
  return segment;

}// End of fitseq()


// 5. Batch tracing paths
static vector<vector<vector<double>>> batchtracepaths (
  vector<vector<vector<double>>> internodepaths,
  double ltres,
  double qtres)
{
  vector<vector<vector<double>>> btracedpaths;
  for(size_t k=0; k<internodepaths.size(); k++)
    btracedpaths.push_back(tracepath(internodepaths[k],ltres,qtres) );
  return btracedpaths;
}


// 5. Batch tracing layers
static vector<vector<vector<vector<double>>>> batchtracelayers (
  vector<vector<vector<vector<double>>>> binternodes,
  double ltres,
  double qtres)
{
  vector<vector<vector<vector<double>>>> btbis;
  for(size_t k=0; k<binternodes.size(); k++)
    btbis.push_back( batchtracepaths( binternodes[k],ltres,qtres) );
  return btbis;
}


////////////////////////////////////////////////////////////
//
//  SVG Drawing functions
//
////////////////////////////////////////////////////////////

static double roundtodec (double val, double places)
{
  return round(val*pow(10,places))/pow(10,places);
}

// Getting SVG path element string from a traced path
static void svgpathstring (stringstream& sb,
                           const string& desc,
                           const vector<vector<double>>& segments,
                           const string& colorstr,
                           const map<string,double>& options)
{
  double scale = options.at("scale"), lcpr = options.at("lcpr"), qcpr = options.at("qcpr"), roundcoords = (double) floor(options.at("roundcoords"));

  // Path
  sb << "<path "
     << desc
     << colorstr
     << " d=\""
     << "M "
     << segments[0][1]*scale
     << " "
     << segments[0][2]*scale
     << " ";

  if( roundcoords == -1 )
  {
    for(size_t pcnt=0;pcnt<segments.size();pcnt++)
    {
      if(segments[pcnt][0]==1.0)
        sb << "L "
           << segments[pcnt][3]*scale
           << " "
           << segments[pcnt][4]*scale
           << " ";
      else
        sb << "Q "
           << segments[pcnt][3]*scale
           << " "
           << segments[pcnt][4]*scale
           << " "
           << segments[pcnt][5]*scale
           << " "
           << segments[pcnt][6]*scale
           << " ";
    }
  }
  else
  {
    for(size_t pcnt=0;pcnt<segments.size();pcnt++)
    {
      if(segments[pcnt][0]==1.0)
      {
        sb << "L "
           << roundtodec(double(segments[pcnt][3]*scale),roundcoords) << " "
           << roundtodec(double(segments[pcnt][4]*scale),roundcoords) << " ";
      }
      else
      {
        sb << "Q "
           << roundtodec(double(segments[pcnt][3]*scale),roundcoords) << " "
           << roundtodec(double(segments[pcnt][4]*scale),roundcoords) << " "
           << roundtodec(double(segments[pcnt][5]*scale),roundcoords) << " "
           << roundtodec(double(segments[pcnt][6]*scale),roundcoords) << " ";
      }
    }
  }// End of roundcoords check

  sb << "Z\" />";

  // Rendering control points
  for(size_t pcnt=0;pcnt<segments.size();pcnt++)
  {
    if((lcpr>0)&&(segments[pcnt][0]==1.0))
    {
      sb <<  "<circle cx=\""
         << segments[pcnt][3]*scale
         << "\" cy=\""
         << segments[pcnt][4]*scale
         << "\" r=\""
         << lcpr
         << "\" fill=\"white\" stroke-width=\""
         << lcpr*0.2
         << "\" stroke=\"black\" />";
    }
    if((qcpr>0)&&(segments[pcnt][0]==2.0))
    {
      sb <<  "<circle cx=\""
         << segments[pcnt][3]*scale
         << "\" cy=\""
         << segments[pcnt][4]*scale
         << "\" r=\""
         << qcpr
         << "\" fill=\"cyan\" stroke-width=\""
         << qcpr*0.2
         << "\" stroke=\"black\" />";
      sb <<  "<circle cx=\""
         << segments[pcnt][5]*scale
         << "\" cy=\""
         << segments[pcnt][6]*scale
         << "\" r=\""
         << qcpr
         << "\" fill=\"white\" stroke-width=\""
         << qcpr*0.2
         << "\" stroke=\"black\" />";
      sb <<  "<line x1=\""
         << segments[pcnt][1]*scale
         << "\" y1=\""
         << segments[pcnt][2]*scale
         << "\" x2=\""
         << segments[pcnt][3]*scale
         << "\" y2=\""
         << segments[pcnt][4]*scale
         << "\" stroke-width=\""
         << qcpr*0.2
         << "\" stroke=\"cyan\" />";
      sb <<  "<line x1=\""
         << segments[pcnt][3]*scale
         << "\" y1=\""
         << segments[pcnt][4]*scale
         << "\" x2=\""
         << segments[pcnt][5]*scale
         << "\" y2=\""
         << segments[pcnt][6]*scale
         << "\" stroke-width=\""
         << qcpr*0.2
         << "\" stroke=\"cyan\" />";
    }// End of quadratic control points
  }

}// End of svgpathstring()


// Converting tracedata to an SVG string, paths are drawn according to a Z-index
// the optional lcpr and qcpr are linear and quadratic control point radiuses
static string getsvgstring (IndexedImage ii, map<string,double> options)
{
  stringstream svgstr;

  options = checkoptions(options);
  // SVG start
  int w = (int) (ii.width * options.at("scale")), h = (int) (ii.height * options.at("scale"));
  string viewboxorviewport;
  if (options.at("viewbox")!=0)
    viewboxorviewport = format("viewBox=\"0 0 {} {}",w,h);
  else
    viewboxorviewport = format("width=\"{}\" height=\"{}\" ",w,h);

  svgstr << "<svg " << viewboxorviewport << "version=\"1.1\" xmlns=\"http://www.w3.org/2000/svg\" ";

  if(options.at("desc")!=0)
  {
    svgstr << "desc=\"Created with ImageTracerC++ version "
           << versionnumber+"\" ";
  }
  svgstr << ">";

  // creating Z-index
  map <double, vector<int>> zindex;
  double label;

  // Layer loop
  for(size_t k=0; k<ii.layers.size(); k++)
  {
    // Path loop
    for(size_t pcnt=0; pcnt<ii.layers[k].size(); pcnt++)
    {
      // Label (Z-index key) is the startpoint of the path, linearized
      label = (ii.layers[k][pcnt][0][2] * w) + ii.layers[k][pcnt][0][1];
      // Creating new list if required
      if(!zindex.count(label))
        zindex[label] = vector<int>(2,0);

      // Adding layer and path number to list
      zindex[label][0] = k;
      zindex[label][1] = pcnt;
    }// End of path loop

  }// End of layer loop

  // Sorting Z-index is not required, TreeMap is sorted automatically

  // Drawing
  // Z-index loop
  string thisdesc = "";
  for(auto& [key, value] : zindex)
  {
    if(options.at("desc")!=0)
      thisdesc = format("desc=\"l {} p {}\" ", value[0], value[1]);
    else
      thisdesc = ""; 

    svgpathstring(svgstr,
        thisdesc,
        ii.layers[value[0]][value[1]],
        tosvgcolorstr(ii.palette[value[0]]),
        options);
  }

  // SVG End
  svgstr << "</svg>";

  return svgstr.str();

}// End of getsvgstring()

static string tosvgcolorstr (const array<uint8_t,4>& c)
{
  return format("fill=\"rgb({},{},{})\" stroke=\"rgb({},{},{})\" stroke-width=\"1\" opacity=\"{}\"",
                c[0],c[1],c[2],
                c[0],c[1],c[2],
                c[3]/255);
}


// Gaussian kernels for blur
static constexpr double gks[5][11] = {
  {0.27901,0.44198,0.27901,0,0,0,0,0,0,0,0},
  {0.135336,0.228569,0.272192,0.228569,0.135336,0,0,0,0,0,0},
  {0.086776,0.136394,0.178908,0.195843,0.178908,0.136394,0.086776,0,0,0,0},
  {0.063327,0.093095,0.122589,0.144599,0.152781,0.144599,0.122589,0.093095,0.063327,0,0},
  {0.049692,0.069304,0.089767,0.107988,0.120651,0.125194,0.120651,0.107988,0.089767,0.069304,0.049692} };


// Selective Gaussian blur for preprocessing
static ImageData blur (ImageData imgd, double rad, double del)
{
  int i,j,k,d,idx;
  double racc,gacc,bacc,aacc,wacc;
  ImageData imgd2(imgd.width,imgd.height,
                  vector<uint8_t>(imgd.width*imgd.height*4,0));

  // radius and delta limits, this kernel
  int radius = (int)floor(rad);
  if(radius<1)
    return imgd;
  if(radius>5)
    radius = 5; 
  int delta = (int)abs(del);
  if (delta>1024)
    delta = 1024; 
  const double* thisgk = gks[radius-1];

  // loop through all pixels, horizontal blur
  for( j=0; j < imgd.height; j++ )
  {
    for( i=0; i < imgd.width; i++ )
    {

      racc = 0; gacc = 0; bacc = 0; aacc = 0; wacc = 0;
      // gauss kernel loop
      for( k = -radius; k < (radius+1); k++)
      {
        // add weighted color values
        if( ((i+k) > 0) && ((i+k) < imgd.width) )
        {
          idx = ((j*imgd.width)+i+k)*4;
          racc += imgd.data[idx  ] * thisgk[k+radius];
          gacc += imgd.data[idx+1] * thisgk[k+radius];
          bacc += imgd.data[idx+2] * thisgk[k+radius];
          aacc += imgd.data[idx+3] * thisgk[k+radius];
          wacc += thisgk[k+radius];
        }
      }
      // The new pixel
      idx = ((j*imgd.width)+i)*4;
      imgd2.data[idx  ] = (uint8_t) floor(racc / wacc);
      imgd2.data[idx+1] = (uint8_t) floor(gacc / wacc);
      imgd2.data[idx+2] = (uint8_t) floor(bacc / wacc);
      imgd2.data[idx+3] = (uint8_t) floor(aacc / wacc);

    }// End of width loop
  }// End of horizontal blur

  // copying the half blurred imgd2
  vector<uint8_t> himgd(imgd2.data);

  // loop through all pixels, vertical blur
  for( j=0; j < imgd.height; j++ )
  {
    for( i=0; i < imgd.width; i++ )
    {

      racc = 0; gacc = 0; bacc = 0; aacc = 0; wacc = 0;
      // gauss kernel loop
      for( k = -radius; k < (radius+1); k++)
      {
        // add weighted color values
        if( ((j+k) > 0) && ((j+k) < imgd.height) )
        {
          idx = (((j+k)*imgd.width)+i)*4;
          racc += himgd[idx  ] * thisgk[k+radius];
          gacc += himgd[idx+1] * thisgk[k+radius];
          bacc += himgd[idx+2] * thisgk[k+radius];
          aacc += himgd[idx+3] * thisgk[k+radius];
          wacc += thisgk[k+radius];
        }
      }
      // The new pixel
      idx = ((j*imgd.width)+i)*4;
      imgd2.data[idx  ] = (uint8_t) floor(racc / wacc);
      imgd2.data[idx+1] = (uint8_t) floor(gacc / wacc);
      imgd2.data[idx+2] = (uint8_t) floor(bacc / wacc);
      imgd2.data[idx+3] = (uint8_t) floor(aacc / wacc);

    }// End of width loop
  }// End of vertical blur

  // Selective blur: loop through all pixels
  for( j=0; j < imgd.height; j++ )
  {
    for( i=0; i < imgd.width; i++ )
    {
      idx = ((j*imgd.width)+i)*4;
      // d is the difference between the blurred and the original pixel
      d = abs(imgd2.data[idx  ] - imgd.data[idx  ]) + abs(imgd2.data[idx+1] - imgd.data[idx+1]) +
          abs(imgd2.data[idx+2] - imgd.data[idx+2]) + abs(imgd2.data[idx+3] - imgd.data[idx+3]);
      // selective blur: if d>delta, put the original pixel back
      if(d>delta)
      {
        imgd2.data[idx  ] = imgd.data[idx  ];
        imgd2.data[idx+1] = imgd.data[idx+1];
        imgd2.data[idx+2] = imgd.data[idx+2];
        imgd2.data[idx+3] = imgd.data[idx+3];
      }
    }
  }// End of Selective blur

  return imgd2;

}// End of blur()

static string toLowerCase(const string& str)
{
  string ret;

  for (char c : str)
    // Asciilower
    if (c <= 'Z' && c >= 'A')
      ret +=  c - ('Z' - 'z');
    else
      ret += c;

  return ret;
}

static int arraycontains (const vector<string>& arr, const string& str)
{
  for(size_t j=0; j<arr.size(); j++ )
  {
    if(toLowerCase(arr[j]) == str)
      return j;
  } return -1;
}

static double parsenext (vector<string>& arr, int i)
{
  if(i<((int)arr.size()-1))
    return atof(arr[i+1].c_str()); // Ignore errors at the moment
  return -1;
}

// Saving a string as a file
static void savestring (const string& filename, const string& str)
{
  ofstream file(filename);

  if (file.good())
    file << str;

  file.close();
}

int main (int argc, char *argv[])
{
  vector<string> args;
  for (int i=1; i<argc; i++)
    args.push_back(argv[i]);
  
  if(args.size()<1)
  {
    print("ERROR: there's no input filename. Basic usage: \n\n"
          "ImageTracer <filename>\n\n"
          "or\n\n"
          "ImageTracer --help\n");
  }
  else if(arraycontains(args,"--help")>-1)
  {
    print("Example usage:\n\n"
          "ImageTracer <filename> outfilename test.svg "
          "ltres 1 qtres 1 pathomit 8 colorsampling 1 numberofcolors 16 mincolorratio 0.02 colorquantcycles 3 "
          "scale 1 simplifytolerance 0 roundcoords 1 lcpr 0 qcpr 0 desc 1 viewbox 0 blurradius 0 blurdelta 20 \n"
          "\nOnly <filename> is mandatory, if some of the other optional parameters are missing, they will be set to these defaults. "
          "\nWarning: if outfilename is not specified, then <filename>.svg will be overwritten."
          "\nSee https://github.com/dov/ImageTracerCpp for details. \nThis is version {}\n", versionnumber);
  }
  else
  {
    // Parameter parsing
    string outfilename = args[0]+".svg";
    map<string,double> options;
    string parameternames[] = {"ltres","qtres","pathomit","colorsampling","numberofcolors","mincolorratio","colorquantcycles","scale","simplifytolerance","roundcoords","lcpr","qcpr","desc","viewbox","blurradius","blurdelta","outfilename"};
    int j = -1; double f = -1;
    for (const string& parametername : parameternames)
    {
      j = arraycontains(args,parametername);
      if(j>-1)
      {
        if(parametername=="outfilename")
        {
          if( j < ((int)args.size()-1))
            outfilename = args[j+1];
        }
        else
        {
          f = parsenext(args,j);
          if(f>-1)
            options[parametername] = f; 
        }
      }
    }// End of parameternames loop
    
    // Loading image, tracing, rendering SVG, saving SVG file
    string svgstring = imageToSVG(args[0],options);
    savestring(outfilename,svgstring);

  }// End of parameter parsing and processing
}// End of main()

  
