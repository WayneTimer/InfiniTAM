// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMMainEngine.h"

using namespace ITMLib::Engine;

ITMMainEngine::ITMMainEngine(const ITMLibSettings *settings, const ITMRGBDCalib *calib, Vector2i imgSize_rgb, Vector2i imgSize_d)
{
	// create all the things required for marching cubes and mesh extraction
	// - uses additional memory (lots!)
	static const bool createMeshingEngine = true;

	if ((imgSize_d.x == -1) || (imgSize_d.y == -1)) imgSize_d = imgSize_rgb;

	this->settings = settings;

	this->scene = new ITMScene<ITMVoxel, ITMVoxelIndex>(&(settings->sceneParams), settings->useSwapping, 
		settings->deviceType == ITMLibSettings::DEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU);

	meshingEngine = NULL;
	switch (settings->deviceType)
	{
	case ITMLibSettings::DEVICE_CPU:
		lowLevelEngine = new ITMLowLevelEngine_CPU();
		viewBuilder = new ITMViewBuilder_CPU(calib);
		visualisationEngine = new ITMVisualisationEngine_CPU<ITMVoxel, ITMVoxelIndex>(scene);
		if (createMeshingEngine) meshingEngine = new ITMMeshingEngine_CPU<ITMVoxel, ITMVoxelIndex>();
		break;
	case ITMLibSettings::DEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
		lowLevelEngine = new ITMLowLevelEngine_CUDA();
		viewBuilder = new ITMViewBuilder_CUDA(calib);
		visualisationEngine = new ITMVisualisationEngine_CUDA<ITMVoxel, ITMVoxelIndex>(scene);
		if (createMeshingEngine) meshingEngine = new ITMMeshingEngine_CUDA<ITMVoxel, ITMVoxelIndex>();
#endif
		break;
	case ITMLibSettings::DEVICE_METAL:
#ifdef COMPILE_WITH_METAL
		lowLevelEngine = new ITMLowLevelEngine_Metal();
		viewBuilder = new ITMViewBuilder_Metal(calib);
		visualisationEngine = new ITMVisualisationEngine_Metal<ITMVoxel, ITMVoxelIndex>(scene);
		if (createMeshingEngine) meshingEngine = new ITMMeshingEngine_CPU<ITMVoxel, ITMVoxelIndex>();
#endif
		break;
	}

	mesh = NULL;
	if (createMeshingEngine) mesh = new ITMMesh(settings->deviceType == ITMLibSettings::DEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU);

	Vector2i trackedImageSize = ITMTrackingController::GetTrackedImageSize(settings, imgSize_rgb, imgSize_d);

	renderState_live = visualisationEngine->CreateRenderState(trackedImageSize);
	renderState_freeview = NULL; //will be created by the visualisation engine

	denseMapper = new ITMDenseMapper<ITMVoxel, ITMVoxelIndex>(settings);
	denseMapper->ResetScene(scene);

	imuCalibrator = new ITMIMUCalibrator_iPad();
	tracker = ITMTrackerFactory<ITMVoxel, ITMVoxelIndex>::Instance().Make(trackedImageSize, settings, lowLevelEngine, imuCalibrator, scene);
	trackingController = new ITMTrackingController(tracker, visualisationEngine, lowLevelEngine, settings);

	trackingState = trackingController->BuildTrackingState(trackedImageSize);
	tracker->UpdateInitialPose(trackingState);

	view = NULL; // will be allocated by the view builder

	fusionActive = true;
	mainProcessingActive = true;
}

ITMMainEngine::~ITMMainEngine()
{
	delete renderState_live;
	if (renderState_freeview!=NULL) delete renderState_freeview;

	delete scene;

	delete denseMapper;
	delete trackingController;

	delete tracker;
	delete imuCalibrator;

	delete lowLevelEngine;
	delete viewBuilder;

	delete trackingState;
	if (view != NULL) delete view;

	delete visualisationEngine;

	if (meshingEngine != NULL) delete meshingEngine;

	if (mesh != NULL) delete mesh;
}

ITMMesh* ITMMainEngine::UpdateMesh(void)
{
	if (mesh != NULL) meshingEngine->MeshScene(mesh, scene);
	return mesh;
}

void ITMMainEngine::SaveSceneToMesh(const char *objFileName)
{
	if (mesh == NULL) return;
	meshingEngine->MeshScene(mesh, scene);
	mesh->WriteSTL(objFileName);
}

// by Timer
void ITMMainEngine::ProcessFrame(ITMUChar4Image *rgbImage, ITMFloatImage *rawDepthImage, ITMIMUMeasurement *imuMeasurement, char* depth_file_name)  // by Timer
//void ITMMainEngine::ProcessFrame(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, ITMIMUMeasurement *imuMeasurement, char* depth_file_name)  // by Timer
{
	// prepare image and turn it into a depth image
	if (imuMeasurement==NULL)
    {
        puts("ITMMainEngine.cpp: IMU == NULL");
        printf("depth_file_name: %s\n", depth_file_name);
        // by Timer
        viewBuilder->float_UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter,settings->modelSensorNoise, depth_file_name);  // by Timer
        //viewBuilder->my_UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter,settings->modelSensorNoise, depth_file_name);  // by Timer
    }
	else
    {
        puts("In IMU mode ??? Error.");
        //viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter, imuMeasurement);
    }

	if (!mainProcessingActive) return;

	
	// tracking
	trackingController->Track(trackingState, view);
	std::cout<<"origin trackingState->pose_d->GetM():"<<std::endl;
	std::cout<<trackingState->pose_d->GetM()<<std::endl;	


    // by Timer
    // save pose
    Matrix3f rotation;
    Vector3f translation;
    int i;
    for (i=strlen(depth_file_name)-1;i>=0;i--)
        if (depth_file_name[i]=='.') break;
    char pose_file_name[100];
    memset(pose_file_name,0,sizeof(pose_file_name));
    strcpy(pose_file_name,depth_file_name);
/*
	int count = 0;
	int base = 1;
	int val = 0;
	for (int j=i-1;count<4;j--)
	{
		val += (pose_file_name[j]-'0') * base;
		base *= 10;
		count++;
	}
	val--;
	count = 0;
	for (int j=i-1;count<4;j--)
	{
		pose_file_name[j] = '0' + (val%10);
		val/=10;
		count++;
	}
*/
    pose_file_name[i+1] = 't';
    pose_file_name[i+2] = 'x';
    pose_file_name[i+3] = 't';
    printf("pose file name: %s\n",pose_file_name);

    FILE *pose_file;
/*
    rotation = trackingState->pose_d->GetR();
    translation = trackingState->pose_d->GetT();
    pose_file = fopen(pose_file_name,"w");
    fprintf(pose_file,"%f %f %f\n",rotation.m00,rotation.m01,rotation.m02);
    fprintf(pose_file,"%f %f %f\n",rotation.m10,rotation.m11,rotation.m12);
    fprintf(pose_file,"%f %f %f\n",rotation.m20,rotation.m21,rotation.m22);
    fprintf(pose_file,"%f %f %f\n",translation.x,translation.y,translation.z);
*/

    // use pose from files
    pose_file = fopen(pose_file_name,"r");
    fscanf(pose_file,"%f",&rotation.m00);
    fscanf(pose_file,"%f",&rotation.m01);
    fscanf(pose_file,"%f",&rotation.m02);

    fscanf(pose_file,"%f",&rotation.m10);
    fscanf(pose_file,"%f",&rotation.m11);
    fscanf(pose_file,"%f",&rotation.m12);

    fscanf(pose_file,"%f",&rotation.m20);
    fscanf(pose_file,"%f",&rotation.m21);
    fscanf(pose_file,"%f",&rotation.m22);

    fscanf(pose_file,"%f",&translation.x);
    fscanf(pose_file,"%f",&translation.y);
    fscanf(pose_file,"%f",&translation.z);  // it is R_k^0, T_k^0


    // R_k^0 -> R_0^k:   seem correct for InfiniTAM
    Matrix3f R_0_k;
    Vector3f T_0_k, tmp;
    R_0_k = rotation.t();
    tmp = R_0_k * translation;
	T_0_k = - tmp;
/*
    trackingState->pose_d->SetRT(R_0_k, T_0_k);
	trackingState->pose_d->Coerce();
*/
	// --- right2left ---
	Matrix3f right2left;
	Vector3f T_right2left(0.0f,0.0f,0.0f);
	right2left.m00 = 1.0;
	right2left.m01 = 0.0;
	right2left.m02 = 0.0;

	right2left.m10 = 0.0;
	right2left.m11 = 0.0;
	right2left.m12 = 1.0;

	right2left.m20 = 0.0;
	right2left.m21 = 1.0;
	right2left.m22 = 0.0;
	ITMPose* r2l = new ITMPose();
	r2l->SetRT(right2left, T_right2left);
	// --- right2left ---





	ITMPose* my_state = new ITMPose();
	my_state->SetRT(R_0_k, T_0_k);

/*
ITMPose* left = new ITMPose();
left->SetM(r2l->GetM() * my_state->GetM());
my_state->SetFrom(left);
*/



	my_state->Coerce();
	std::cout<<"R_0^k, T_0^k"<<std::endl;
	std::cout<<"my_state->GetM():"<<std::endl;
	std::cout<<my_state->GetM()<<std::endl;

	ITMPose* new_state = new ITMPose();
	if (!use_our_state)
	{
		first_pose->SetFrom(my_state);
		use_our_state = true;
	}
	else
	{	
		new_state->SetM(my_state->GetM() * first_pose->GetInvM());
		new_state->Coerce();
		std::cout<<"new_state->GetM():"<<std::endl;  // This!
		std::cout<<new_state->GetM()<<std::endl;
		std::cout<<"new_state->GetInvM():"<<std::endl;
		std::cout<<new_state->GetInvM()<<std::endl;

		ITMPose* relative_pose = new ITMPose();
		relative_pose->SetM(new_state->GetM() * last_pose->GetInvM());
		relative_pose->Coerce();
		std::cout<<"relative_pose->GetM():"<<std::endl;
		std::cout<<relative_pose->GetM()<<std::endl;

		last_pose->SetM(new_state->GetM());
	}


    fclose(pose_file);

	// fusion
	if (fusionActive) denseMapper->ProcessFrame(view, trackingState, scene, renderState_live);

	// raycast to renderState_live for tracking and free visualisation
	trackingController->Prepare(trackingState, view, renderState_live);


	// by Timer
	// trackingState is the current state - 1
	/*
    trackingState->pose_d->SetFrom(new_state);
	//use_our_state = true;	
	*/
}

Vector2i ITMMainEngine::GetImageSize(void) const
{
	return renderState_live->raycastImage->noDims;
}

void ITMMainEngine::GetImage(ITMUChar4Image *out, GetImageType getImageType, ITMPose *pose, ITMIntrinsics *intrinsics)
{
	if (view == NULL) return;

	out->Clear();

	switch (getImageType)
	{
	case ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_RGB:
		out->ChangeDims(view->rgb->noDims);
		if (settings->deviceType == ITMLibSettings::DEVICE_CUDA) 
			out->SetFrom(view->rgb, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
		else out->SetFrom(view->rgb, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
		break;
	case ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH:
		out->ChangeDims(view->depth->noDims);
		if (settings->trackerType==ITMLib::Objects::ITMLibSettings::TRACKER_WICP)
		{
			if (settings->deviceType == ITMLibSettings::DEVICE_CUDA) view->depthUncertainty->UpdateHostFromDevice();
			ITMVisualisationEngine<ITMVoxel, ITMVoxelIndex>::WeightToUchar4(out, view->depthUncertainty);
		}
		else
		{
			if (settings->deviceType == ITMLibSettings::DEVICE_CUDA) view->depth->UpdateHostFromDevice();
			ITMVisualisationEngine<ITMVoxel, ITMVoxelIndex>::DepthToUchar4(out, view->depth);
		}

		break;
	case ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST:
	{
		ORUtils::Image<Vector4u> *srcImage = renderState_live->raycastImage;
		out->ChangeDims(srcImage->noDims);
		if (settings->deviceType == ITMLibSettings::DEVICE_CUDA)
			out->SetFrom(srcImage, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
		else out->SetFrom(srcImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);	
		break;
	}
	case ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_SHADED:
	case ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME:
	case ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL:
	{
		IITMVisualisationEngine::RenderImageType type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE;
		if (getImageType == ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME) type = IITMVisualisationEngine::RENDER_COLOUR_FROM_VOLUME;
		else if (getImageType == ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL) type = IITMVisualisationEngine::RENDER_COLOUR_FROM_NORMAL;
		if (renderState_freeview == NULL) renderState_freeview = visualisationEngine->CreateRenderState(out->noDims);

		visualisationEngine->FindVisibleBlocks(pose, intrinsics, renderState_freeview);
		visualisationEngine->CreateExpectedDepths(pose, intrinsics, renderState_freeview);
		visualisationEngine->RenderImage(pose, intrinsics, renderState_freeview, renderState_freeview->raycastImage, type);

		if (settings->deviceType == ITMLibSettings::DEVICE_CUDA)
			out->SetFrom(renderState_freeview->raycastImage, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
		else out->SetFrom(renderState_freeview->raycastImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
		break;
	}
	case ITMMainEngine::InfiniTAM_IMAGE_UNKNOWN:
		break;
	};
}

void ITMMainEngine::turnOnIntegration() { fusionActive = true; }
void ITMMainEngine::turnOffIntegration() { fusionActive = false; }
void ITMMainEngine::turnOnMainProcessing() { mainProcessingActive = true; }
void ITMMainEngine::turnOffMainProcessing() { mainProcessingActive = false; }
