// Pixelate.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include<iostream>
#include<C:\Pixelate\BeliefState\BeliefState.h>

using namespace std;
using namespace BS;

int main()
{
	BeliefState state;
	state.update();
	cout<<"testing"<<state.ballPosX<<endl;
	getchar();
	return 0;
}

