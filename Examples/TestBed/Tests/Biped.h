#pragma once
#include "Box2D.h"
#include "BipedDef.h"

class Biped
{
public:
	void SetMotorTorque(float);
	float GetMass();
	b2Vec2 GetCenterPosition();
	Biped(b2World*, const BipedDef*);
	~Biped(void);

	b2Body				*LFoot, *RFoot, *LCalf, *RCalf, *LThigh, *RThigh,
						*Pelvis, *Stomach, *Chest, *Neck, *Head,
						*LUpperArm, *RUpperArm, *LForearm, *RForearm, *LHand, *RHand;

	b2RevoluteJoint		*LAnkle, *RAnkle, *LKnee, *RKnee, *LHip, *RHip, 
						*LowerAbs, *UpperAbs, *LowerNeck, *UpperNeck,
						*LShoulder, *RShoulder, *LElbow, *RElbow, *LWrist, *RWrist;
};
