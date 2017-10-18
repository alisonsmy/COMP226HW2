//-----------------------------------------------------------------------------
// HW02 - Builds with SKA Version 4.0
//-----------------------------------------------------------------------------
// AnimationControl.cpp
//    Animation controller for multiple characters.
//    This is intended to be used for HW2 - COMP 259 fall 2017.
//-----------------------------------------------------------------------------
// SKA configuration
#include <Core/SystemConfiguration.h>
// C/C++ libraries
#include <cstdio>
#include <complex>
#include <vector>
// SKA modules
#include <Core/Utilities.h>
//#include <Animation/RawMotionController.h>
#include <Animation/AnimationException.h>
#include <Animation/Skeleton.h>
#include <DataManagement/DataManager.h>
#include <DataManagement/DataManagementException.h>
#include <DataManagement/BVH_Reader.h>
// local application
#include "AppConfig.h"
#include "AnimationControl.h"
#include "RenderLists.h"
#include "OpenMotionSequenceController.h"

// global single instance of the animation controller
AnimationControl anim_ctrl;

enum MOCAP_TYPE { BVH, AMC };

struct LoadSpec {
	MOCAP_TYPE mocap_type;
	float scale;
	Color color;
	string motion_file;
	string skeleton_file;
	LoadSpec(MOCAP_TYPE _mocap_type, float _scale, Color& _color, string& _motion_file, string& _skeleton_file=string(""))
		: mocap_type(_mocap_type), scale(_scale), color(_color), motion_file(_motion_file), skeleton_file(_skeleton_file) { }
};

const short NUM_CHARACTERS = 2;
LoadSpec load_specs[NUM_CHARACTERS] = {
	//LoadSpec(AMC, 1.0f, Color(0.8f,0.4f,0.8f), string("02/02_01.amc"), string("02/02.asf")),
	//LoadSpec(AMC, 1.0f, Color(1.0f,0.4f,0.3f), string("16/16_55.amc"), string("16/16.asf")),
	//LoadSpec(BVH, 0.2f, Color(0.0f,1.0f,0.0f), string("avoid/Avoid 9.bvh"))

	// character 1 (purple) - fast walk
	//LoadSpec(BVH, 0.2f, Color(0.8f,0.4f,0.8f), string("locomotion_takes_Oct03/Take 2017-10-03 03.42.17 PM.bvh"))
	// character 2 (red-orange) - medium walk
	LoadSpec(BVH, 0.2f, Color(1.0f,0.4f,0.3f), string("locomotion_takes_Oct03/Take 2017-10-03 03.43.40 PM.bvh")),
	// character 3 (green) - slow walk
	LoadSpec(BVH, 0.2f, Color(0.0f,1.0f,0.0f), string("locomotion_takes_Oct03/Take 2017-10-03 03.55.32 PM.bvh"))
};

Object* createMarkerBox(Vector3D position, Color _color)
{
	ModelSpecification markerspec("Box", _color);
	markerspec.addSpec("length", "0.5");
	markerspec.addSpec("width", "0.5");
	markerspec.addSpec("height", "0.5");
	Object* marker = new Object(markerspec,	position, Vector3D(0.0f, 0.0f, 0.0f));
	return marker;
}

AnimationControl::AnimationControl() 
	: ready(false), run_time(0.0f), 
	global_timewarp(1.0f),
	next_marker_time(0.1f), marker_time_interval(0.1f), max_marker_time(20.0f)
{ } 

AnimationControl::~AnimationControl()	
{		
	for (unsigned short c=0; c<characters.size(); c++)
		if (characters[c] != NULL) delete characters[c]; 
}

void AnimationControl::restart()
{ 
	render_lists.eraseErasables();
	run_time = 0; 
	updateAnimation(0.0f); 
	next_marker_time = marker_time_interval;
}

//std::vector<Vector3D> left_toe_position[NUM_CHARACTERS];
std::vector<long> keyframes_frame[NUM_CHARACTERS];  // frame number
std::vector<float> keyframes_time[NUM_CHARACTERS];  // local time
int i, j = 0;
int first_pass_through[NUM_CHARACTERS] = { 0 };
//int slow_skelevalues[4] = { 0.45, -0.55, 1.3, -0.35 };  // in order
//int left_time_pause_foot[NUM_CHARACTERS] = { 0.9f };
//int right_time_pause_foot[NUM_CHARACTERS] = { 0.9f };
bool lr_skel_bool[NUM_CHARACTERS] = { true };  // right = true, left = false
bool AnimationControl::updateAnimation(float _elapsed_time)
{
	if (!ready) return false;

	// the global time warp can be applied directly to the elapsed time between updates
	float warped_elapsed_time = global_timewarp * _elapsed_time;

	run_time += warped_elapsed_time;
	for (unsigned short c = 0; c < characters.size(); c++)
	{	
		OpenMotionSequenceController* controller = (OpenMotionSequenceController*)characters[c]->getMotionController();
		long current_frame = controller->getSequenceFrame();
		
		if (c == 1)  // slower character
		{
			if (characters[c] != NULL) characters[c]->update(run_time);  // no time warp

			if (first_pass_through[c] > 0)
			{
				if (current_frame == keyframes_frame[c].at(j))
				{
					if (j == keyframes_frame[1].size() - 1) j = 0;  // increment through keyframe list
					else j++;
				}
			}
		}
		else  // faster character
		{
			if (first_pass_through[c] == 0)  // still in first pass
			{
				if (characters[c] != NULL) characters[c]->update(run_time);  // no time warp
			}
			else 
			{
				if (characters[c] != NULL) characters[c]->update(run_time / warpTime(i,j));  // apply time warp
				
				if (current_frame == keyframes_frame[c].at(i))
				{
					if (i == keyframes_frame[0].size() - 1) i = 0;  // increment through keyframe list
					else i++;
				}
			}
		}

		// pull local time and frame out of each skeleton's controller
		// (dangerous upcast)
		//OpenMotionSequenceController* controller = (OpenMotionSequenceController*)characters[c]->getMotionController();
		display_data.sequence_time[c] = controller->getSequenceTime();
		display_data.sequence_frame[c] = controller->getSequenceFrame();

		if (display_data.sequence_frame[c] == 0) //not working - never true
		{
			first_pass_through[c]++;
			cout << first_pass_through[c];
		}
		
		/* TODO: detect keyframe */
		Vector3D start, end;
		//characters[c]->getBonePositions("RightToeBase", start, end);
		characters[c]->getBonePositions("RightToeBase", start, end);
		/*
		if (start.y  <= 5 && first_pass_through[c] == 1)
		{
			keyframes_frame[c].push_back(display_data.sequence_frame[c]);
			keyframes_time[c].push_back(display_data.sequence_time[c]);
			cout << "This is time frame: " << keyframes_time[c].back() << "\n";
			cout << "This is model frame: " << keyframes_frame[c].back() << "\n";
			
		}*/




		if (end.y <= 0.45 && end.y >= -1 && first_pass_through[c] == 0 
			&& !lr_skel_bool[c] && (keyframes_time[c].empty() || (controller->getSequenceTime() - keyframes_time[c].back() >= 0.9f)))  //need a better condition - y is never exactly 0
		{
			Color color = Color(0.8f, 0.3f, 0.8f);
			Object* marker = createMarkerBox(end, color);
			render_lists.erasables.push_back(marker);
			//next_marker_time += marker_time_interval;

			keyframes_frame[c].push_back(display_data.sequence_frame[c]);  //getSequenceFrame()
			keyframes_time[c].push_back(display_data.sequence_time[c]);  //getSequenceTime()
				
			//cout << "This is time frame: " << keyframes_time[c].back() << "\n";
			//cout << "This is model frame: " << keyframes_frame[c].back() << "\n";

			//Can assume start of frame 0 is when both feet are on the ground
			//Everytime end.y <= 0 the left foot is on the ground and can use that as a keyframe
			//
			cout << "Right Foot Y: " << end.y << endl;

			lr_skel_bool[c] = true;
		}


		//characters[c]->getBonePositions("LeftToeBase", start, end);

		characters[c]->getBonePositions("LeftToeBase", start, end);
		
		if (end.y <= 1.3 && end.y >= -1 && first_pass_through[c] == 0
			&& lr_skel_bool[c] && ( keyframes_time[c].size() == 0 || (controller->getSequenceTime() - keyframes_time[c].back() >= 0.9f) ))  //need a better condition - y is never exactly 0
		{
			Color color = Color(0.3f, 0.3f, 0.9f);
			Object* marker = createMarkerBox(end, color);
			render_lists.erasables.push_back(marker);
			//next_marker_time += marker_time_interval;
			
			keyframes_frame[c].push_back(display_data.sequence_frame[c]);  //getSequenceFrame()
			keyframes_time[c].push_back(display_data.sequence_time[c]);  //getSequenceTime()
			//cout << "This is time frame: " << keyframes_time[c].back() << "\n";
			//cout << "This is model frame: " << keyframes_frame[c].back() << "\n";
			
			//Can assume start of frame 0 is when both feet are on the ground
			//Everytime end.y <= 0 the left foot is on the ground and can use that as a keyframe
			cout << "Left foot y: " << end.y << endl;
			lr_skel_bool[c] = false;
		}



		//left_toe_position[c].push_back(end);
		//cout << left_toe_position[c=].back() << "\n";
	}
/*
	if (run_time >= next_marker_time && run_time <= max_marker_time)
	{
		Color color = Color(0.8f, 0.3f, 0.3f);
		Vector3D start, end;
		// drop box at left toes of 1st character
		// CAREFUL - bones names are different in different skeletons
		characters[0]->getBonePositions("LeftToeBase", start, end);
		Object* marker = createMarkerBox(end, color);
		render_lists.erasables.push_back(marker);
		next_marker_time += marker_time_interval;
	}
	*/

	return true;
}

float AnimationControl::warpTime(int i, int j)
{
	cout << "time warpppp: " << keyframes_time[1].at(j) / keyframes_time[0].at(i);
	return keyframes_time[1].at(j) / keyframes_time[0].at(i);
}

static Skeleton* buildCharacter(
	Skeleton* _skel, 
	MotionSequence* _ms, 
	Color _bone_color, 
	const string& _description1, 
	const string& _description2,
	vector<Object*>& _render_list)
{
	if ((_skel == NULL) || (_ms == NULL)) return NULL;

	OpenMotionSequenceController* controller = new OpenMotionSequenceController(_ms);

	//! Hack. The skeleton expects a list<Object*>, we're using a vector<Object*>
	list<Object*> tmp;
	_skel->constructRenderObject(tmp, _bone_color);
	list<Object*>::iterator iter = tmp.begin();
	while (iter != tmp.end()) { _render_list.push_back(*iter); iter++; }
	//! EndOfHack.
	
	_skel->attachMotionController(controller);
	_skel->setDescription1(_description1.c_str());
	_skel->setDescription2(_description2.c_str());
	return _skel;
}

void AnimationControl::loadCharacters()
{
	data_manager.addFileSearchPath(AMC_MOTION_FILE_PATH);
	data_manager.addFileSearchPath(BVH_MOTION_FILE_PATH);

	Skeleton* skel = NULL;
	MotionSequence* ms = NULL;
	string descr1, descr2;
	char* filename1 = NULL;
	char* filename2 = NULL;
	Skeleton* character = NULL;
	pair<Skeleton*, MotionSequence*> read_result;

	for (short c = 0; c < NUM_CHARACTERS; c++)
	{
		if (load_specs[c].mocap_type == AMC)
		{
			try
			{
				filename1 = data_manager.findFile(load_specs[c].skeleton_file.c_str());
				if (filename1 == NULL)
				{
					logout << "AnimationControl::loadCharacters: Unable to find character ASF file <" << load_specs[c].skeleton_file << ">. Aborting load." << endl;
					throw BasicException("ABORT 1A");
				}
				filename2 = data_manager.findFile(load_specs[c].motion_file.c_str());
				if (filename2 == NULL)
				{
					logout << "AnimationControl::loadCharacters: Unable to find character AMC file <" << load_specs[c].motion_file << ">. Aborting load." << endl;
					throw BasicException("ABORT 1B");
				}
				try {
					read_result = data_manager.readASFAMC(filename1, filename2);
				}
				catch (const DataManagementException& dme)
				{
					logout << "AnimationControl::loadCharacters: Unable to load character data files. Aborting load." << endl;
					logout << "   Failure due to " << dme.msg << endl;
					throw BasicException("ABORT 1C");
				}
			}
			catch (BasicException&) {}
		}
		else if (load_specs[c].mocap_type == BVH)
		{
			try
			{
				filename1 = data_manager.findFile(load_specs[c].motion_file.c_str());
				if (filename1 == NULL)
				{
					logout << "AnimationControl::loadCharacters: Unable to find character BVH file <" << load_specs[c].motion_file << ">. Aborting load." << endl;
					throw BasicException("ABORT 2A");
				}
				try
				{
					read_result = data_manager.readBVH(filename1);
				}
				catch (const DataManagementException& dme)
				{
					logout << "AnimationControl::loadCharacters: Unable to load character data files. Aborting load." << endl;
					logout << "   Failure due to " << dme.msg << endl;
					throw BasicException("ABORT 2C");
				}
			}
			catch (BasicException&) {}
		}

		try
		{
			skel = read_result.first;
			ms = read_result.second;
			
			skel->scaleBoneLengths(load_specs[c].scale);
			ms->scaleChannel(CHANNEL_ID(0, CT_TX), load_specs[c].scale);
			ms->scaleChannel(CHANNEL_ID(0, CT_TY), load_specs[c].scale);
			ms->scaleChannel(CHANNEL_ID(0, CT_TZ), load_specs[c].scale);

			// create a character to link all the pieces together.
			descr1 = string("skeleton: ") + load_specs[c].skeleton_file;
			descr2 = string("motion: ") + load_specs[c].motion_file;

			character = buildCharacter(skel, ms, load_specs[c].color, descr1, descr2, render_lists.bones);
			if (character != NULL) characters.push_back(character);
		}
		catch (BasicException&) {}
		
		strDelete(filename1); filename1 = NULL;
		strDelete(filename2); filename2 = NULL;
	}

	display_data.num_characters = (short)characters.size();
	display_data.sequence_time.resize(characters.size());
	display_data.sequence_frame.resize(characters.size());

	if (characters.size() > 0) ready = true;
}


