// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ImageSourceEngine.h"

#include "../Utils/FileUtils.h"

#include <stdio.h>

using namespace InfiniTAM::Engine;

ImageSourceEngine::ImageSourceEngine(const char *calibFilename)
{
	readRGBDCalib(calibFilename, calib);
}

ImageFileReader::ImageFileReader(const char *calibFilename, const char *rgbImageMask, const char *depthImageMask)
	: ImageSourceEngine(calibFilename)
{
	strncpy(this->rgbImageMask, rgbImageMask, BUF_SIZE);
	strncpy(this->depthImageMask, depthImageMask, BUF_SIZE);

	currentFrameNo = 0;
	cachedFrameNo = -1;

	cached_rgb = NULL;
	cached_depth = NULL;
    my_cached_depth = NULL;
}

ImageFileReader::~ImageFileReader()
{
	delete cached_rgb;
	delete cached_depth;
    delete my_cached_depth;
}


_CPU_AND_GPU_CODE_ inline void my_convertDisparityToDepth(float* d_out, int x, int y, const CONSTPTR(short) *d_in)
{
	int locId = x + y * MY_IMAGE_WIDTH;

	short disparity = d_in[locId];
	float disparity_tmp = 1135.089966f - (float)(disparity);
	float depth;

	if (disparity_tmp == 0) depth = 0.0f;
	else depth = 8.0f * 0.081914f * 573.710022f / disparity_tmp;

	*d_out = (depth > 0) ? depth : -1.0f;
}

void ImageFileReader::loadIntoCache(void)
{
	if (currentFrameNo == cachedFrameNo) return;
	cachedFrameNo = currentFrameNo;

	//TODO> make nicer
	cached_rgb = new ITMUChar4Image(true, false); 
	cached_depth = new ITMShortImage(true, false);
	my_cached_depth = new ITMFloatImage(true, false);

	char str[2048];

	sprintf(str, rgbImageMask, currentFrameNo);

    // by Timer
    {
        char img_file_name[2048];
        int i;
        for (i=strlen(str)-1;i>=0;i--)
            if (str[i]=='/') break;
        i++;

        memset(img_file_name,0,sizeof(img_file_name));
        strcat(img_file_name, INPUT_DIR);
        strcat(img_file_name, str+i);
        i = strlen(img_file_name);
        img_file_name[i-1] = 'g';
        img_file_name[i-2] = 'n';
        img_file_name[i-3] = 'p';
        printf("Cache -> ImageFileReader::getImages:  img_file_name: %s\n",img_file_name);    // .png

	    if (!my_ReadImageFromFile(cached_rgb, img_file_name)) 
	    {
		    delete cached_rgb; cached_rgb = NULL;
		    printf("error reading file '%s'\n", img_file_name);
	    }
    }


/*
	if (!ReadImageFromFile(cached_rgb, str)) 
	{
		delete cached_rgb; cached_rgb = NULL;
		printf("error reading file '%s'\n", str);
	}
*/


	sprintf(str, depthImageMask, currentFrameNo);

    // by Timer
    {
        char depth_file_name[2048];
        int i;
        for (i=strlen(str)-1;i>=0;i--)
            if (str[i]=='/') break;
        i++;

        memset(depth_file_name,0,sizeof(depth_file_name));
        strcat(depth_file_name, INPUT_DIR);
        strcat(depth_file_name, str+i);
        i = strlen(depth_file_name);
        depth_file_name[i-1] = 'm';
        depth_file_name[i-2] = 'g';
        depth_file_name[i-3] = 'p';
        printf("Cache -> ImageFileReader::getImages:  depth_file_name: %s\n",depth_file_name);    // .png

	    if (!float_ReadImageFromFile(my_cached_depth, depth_file_name)) 
	    {
		    delete my_cached_depth; my_cached_depth = NULL;
		    printf("error reading file '%s'\n", depth_file_name);
	    }
    }
    return;


	if (!ReadImageFromFile(cached_depth, str)) 
	{
		delete cached_depth; cached_depth = NULL;
		printf("error reading file '%s'\n", str);
	}
    else // by Timer
    {
        int i;
        for (i=strlen(str)-1;i>=0;i--)
            if (str[i]=='/') break;
        i++;
        printf("Reading depth file: %s\n", str+i);

// by Timer
/*
        FILE *depth_file;
        char depth_file_name[100];
        memset(depth_file_name,0,sizeof(depth_file_name));
        strcat(depth_file_name, "/home/timer/work_git/Teddy/depth2pointcloud/depth/");
        strcat(depth_file_name, str+i);
        printf("depth_file_name: %s\n",depth_file_name);    // also .pgm

        depth_file = fopen(depth_file_name, "w");
        if (!depth_file)
            puts("depth file open error!");

    	Vector2i imgSize = cached_depth->noDims;
        printf("Size: x = %d, y = %d\n",imgSize.x,imgSize.y);

	    const short *d_in = cached_depth->GetData(MEMORYDEVICE_CPU);
        float d_out;

	    for (int y = 0; y < MY_IMAGE_HEIGHT; y++)
            for (int x = 0; x < MY_IMAGE_WIDTH; x++)
            {
		        my_convertDisparityToDepth(&d_out, x, y, d_in);
                fprintf(depth_file, "%f ", d_out);
            }

        fclose(depth_file);
*/
    }
}

bool ImageFileReader::hasMoreImages(void)
{
	loadIntoCache();
	return ((cached_rgb!=NULL)&&(cached_depth!=NULL)&&(my_cached_depth!=NULL));
}


// by Timer
void ImageFileReader::my_getImages(ITMUChar4Image *rgb, ITMFloatImage *rawDepth, char* img_file_name, char* depth_file_name)
//void ImageFileReader::my_getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth, char* img_file_name, char* depth_file_name)
{
    puts("Into ImageFileReader::my_getImages.");

	bool bUsedCache = false;
	if (cached_rgb != NULL) {
		rgb->SetFrom(cached_rgb, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
		delete cached_rgb;
		cached_rgb = NULL;
		bUsedCache = true;

        // by Timer
        char str[2048];
		sprintf(str, rgbImageMask, currentFrameNo);
        int i;
        for (i=strlen(str)-1;i>=0;i--)
            if (str[i]=='/') break;
        i++;

        strcat(img_file_name, INPUT_DIR);
        strcat(img_file_name, str+i);
        i = strlen(img_file_name);
        img_file_name[i-1] = 'g';
        img_file_name[i-2] = 'n';
        img_file_name[i-3] = 'p';
        printf("ImageFileReader::getImages:  img_file_name: %s\n",img_file_name);    // .png
	}
	if (my_cached_depth != NULL) {
        puts("Before rawDepth->SetFrom");
		rawDepth->SetFrom(my_cached_depth, ORUtils::MemoryBlock<float>::CPU_TO_CPU);
        puts("After rawDepth->SetFrom");
		delete my_cached_depth;
		my_cached_depth = NULL;
		bUsedCache = true;

        // by Timer
        char str[2048];
		sprintf(str, depthImageMask, currentFrameNo);
        int i;
        for (i=strlen(str)-1;i>=0;i--)
            if (str[i]=='/') break;
        i++;
        printf("Reading depth file: %s\n", str+i);

        strcat(depth_file_name, INPUT_DIR);
        strcat(depth_file_name, str+i);
        printf("ImageFileReader::getImages:  depth_file_name: %s\n",depth_file_name);    // also .pgm
	}
    else
    {
        puts("ImageSouceEngine.cpp -> No my_cached_depth?");
    }

	if (!bUsedCache) {
		char str[2048];

		sprintf(str, rgbImageMask, currentFrameNo);
		if (!ReadImageFromFile(rgb, str)) printf("error reading file '%s'\n", str);

		sprintf(str, depthImageMask, currentFrameNo);
		if (!float_ReadImageFromFile(rawDepth, str)) printf("error reading file '%s'\n", str);
	}

	++currentFrameNo;
}

void ImageFileReader::getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth)
{
	bool bUsedCache = false;
	if (cached_rgb != NULL) {
		rgb->SetFrom(cached_rgb, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);

		delete cached_rgb;
		cached_rgb = NULL;
		bUsedCache = true;
	}
	if (cached_depth != NULL) {
		rawDepth->SetFrom(cached_depth, ORUtils::MemoryBlock<short>::CPU_TO_CPU);
		delete cached_depth;
		cached_depth = NULL;
		bUsedCache = true;
	}

	if (!bUsedCache) {
		char str[2048];

		sprintf(str, rgbImageMask, currentFrameNo);
		if (!ReadImageFromFile(rgb, str)) printf("error reading file '%s'\n", str);

		sprintf(str, depthImageMask, currentFrameNo);

		if (!ReadImageFromFile(rawDepth, str)) printf("error reading file '%s'\n", str);
	}

	++currentFrameNo;
}

Vector2i ImageFileReader::getDepthImageSize(void)
{
	loadIntoCache();
    return my_cached_depth->noDims;  // by Timer
	//return cached_depth->noDims;
}

Vector2i ImageFileReader::getRGBImageSize(void)
{
	loadIntoCache();
	if (cached_rgb != NULL) return cached_rgb->noDims;
    return my_cached_depth->noDims; // by Timer
	//return cached_depth->noDims;
}

CalibSource::CalibSource(const char *calibFilename, Vector2i setImageSize, float ratio)
	: ImageSourceEngine(calibFilename)
{
	this->imgSize = setImageSize;
	this->ResizeIntrinsics(calib.intrinsics_d, ratio);
	this->ResizeIntrinsics(calib.intrinsics_rgb, ratio);
}

void CalibSource::ResizeIntrinsics(ITMIntrinsics &intrinsics, float ratio)
{
	intrinsics.projectionParamsSimple.fx *= ratio;
	intrinsics.projectionParamsSimple.fy *= ratio;
	intrinsics.projectionParamsSimple.px *= ratio;
	intrinsics.projectionParamsSimple.py *= ratio;
	intrinsics.projectionParamsSimple.all *= ratio;
}

RawFileReader::RawFileReader(const char *calibFilename, const char *rgbImageMask, const char *depthImageMask, Vector2i setImageSize, float ratio) 
	: ImageSourceEngine(calibFilename)
{
	this->imgSize = setImageSize;
	this->ResizeIntrinsics(calib.intrinsics_d, ratio);
	this->ResizeIntrinsics(calib.intrinsics_rgb, ratio);
	
	strncpy(this->rgbImageMask, rgbImageMask, BUF_SIZE);
	strncpy(this->depthImageMask, depthImageMask, BUF_SIZE);

	currentFrameNo = 0;
	cachedFrameNo = -1;

	cached_rgb = NULL;
	cached_depth = NULL;
}

void RawFileReader::ResizeIntrinsics(ITMIntrinsics &intrinsics, float ratio)
{
	intrinsics.projectionParamsSimple.fx *= ratio;
	intrinsics.projectionParamsSimple.fy *= ratio;
	intrinsics.projectionParamsSimple.px *= ratio;
	intrinsics.projectionParamsSimple.py *= ratio;
	intrinsics.projectionParamsSimple.all *= ratio;
}

void RawFileReader::loadIntoCache(void)
{
	if (currentFrameNo == cachedFrameNo) return;
	cachedFrameNo = currentFrameNo;

	//TODO> make nicer
	cached_rgb = new ITMUChar4Image(imgSize, MEMORYDEVICE_CPU);
	cached_depth = new ITMShortImage(imgSize, MEMORYDEVICE_CPU);

	char str[2048]; FILE *f; bool success = false;

	sprintf(str, rgbImageMask, currentFrameNo);

	f = fopen(str, "rb");
	if (f)
	{
		size_t tmp = fread(cached_rgb->GetData(MEMORYDEVICE_CPU), sizeof(Vector4u), imgSize.x * imgSize.y, f);
		fclose(f);
		if (tmp == (size_t)imgSize.x * imgSize.y) success = true;
	}
	if (!success)
	{
		delete cached_rgb; cached_rgb = NULL;
		printf("error reading file '%s'\n", str);
	}

	sprintf(str, depthImageMask, currentFrameNo); success = false;
	f = fopen(str, "rb");
	if (f)
	{
        puts("In RawFileReader ??? Error");
		size_t tmp = fread(cached_depth->GetData(MEMORYDEVICE_CPU), sizeof(short), imgSize.x * imgSize.y, f);
		fclose(f);
		if (tmp == (size_t)imgSize.x * imgSize.y) success = true;
	}
	if (!success)
	{
        puts("In RawFileReader ??? Error");
		delete cached_depth; cached_depth = NULL;
		printf("error reading file '%s'\n", str);
	}
}


bool RawFileReader::hasMoreImages(void)
{
	loadIntoCache(); 

	return ((cached_rgb != NULL) || (cached_depth != NULL));
}

void RawFileReader::getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth)
{
	bool bUsedCache = false;

	if (cached_rgb != NULL)
	{
		rgb->SetFrom(cached_rgb, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
		delete cached_rgb;
		cached_rgb = NULL;
		bUsedCache = true;
	}

	if (cached_depth != NULL)
	{
		rawDepth->SetFrom(cached_depth, ORUtils::MemoryBlock<short>::CPU_TO_CPU);
		delete cached_depth;
		cached_depth = NULL;
		bUsedCache = true;
	}

	if (!bUsedCache) this->loadIntoCache();

	++currentFrameNo;
}
