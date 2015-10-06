#include "stdafx.h"
#include<iostream>
#include<C:\Pixelate\BeliefState\BeliefState.h>

using namespace std;
using namespace BS;
namespace BS
{
	BeliefState::BeliefState()
	{
		botPosX=0;
		botPosY=0;
		ballPosX=0;
		ballPosY=0;
		boxPosX=0;
		boxPosY=0;
		
	}
	void BeliefState::update()
	{

	}
	void BeliefState::calc_vel()
	{
		/*
		   calculates the velocity of the bot 
		*/
	}
	void BeliefState::calc_botPos()
	{
		/*
		  1.color detection
		  2.contour finding
		  3.center determination
		  4.update the bot position variables
		*/
	}
	void BeliefState::calc_ballPos()
	{
		/*
		  1.color detection
		  2.contour finding
		  3.center determination
		  4.update the ball position variables
		*/
	}
	void BeliefState::calc_boxPos()
	{
		/*
		  1.color detection
		  2.contour finding
		  3.center determination
		  4.update the box position variables
		*/
	}

	void BeliefState::calc_arrowPos()
	{
		/*
		  1.color detection
		  2.contour finding
		  3.center determination
		  4.update the arrow position variables
		*/
	}
}