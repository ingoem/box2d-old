#include "Biped.h"

Biped::Biped(b2World* w, const BipedDef* bd)				
{
	BipedDef b(*bd);

	// update positions
	b.LFootDef.position += b.position;
	b.RFootDef.position += b.position;
	b.LCalfDef.position += b.position;
	b.RCalfDef.position += b.position;
	b.LThighDef.position += b.position;
	b.RThighDef.position += b.position;
	b.PelvisDef.position += b.position;
	b.StomachDef.position += b.position;
	b.ChestDef.position += b.position;
	b.NeckDef.position += b.position;
	b.HeadDef.position += b.position;
	b.LUpperArmDef.position += b.position;
	b.RUpperArmDef.position += b.position;
	b.LForearmDef.position += b.position;
	b.RForearmDef.position += b.position;
	b.LHandDef.position += b.position;
	b.RHandDef.position += b.position;

	b.LAnkleDef.anchorPoint += b.position;
	b.RAnkleDef.anchorPoint += b.position;
	b.LKneeDef.anchorPoint += b.position;
	b.RKneeDef.anchorPoint += b.position;
	b.LHipDef.anchorPoint += b.position;
	b.RHipDef.anchorPoint += b.position;
	b.LowerAbsDef.anchorPoint += b.position;
	b.UpperAbsDef.anchorPoint += b.position;
	b.LowerNeckDef.anchorPoint += b.position;
	b.UpperNeckDef.anchorPoint += b.position;
	b.LElbowDef.anchorPoint += b.position;
	b.RElbowDef.anchorPoint += b.position;
	b.LShoulderDef.anchorPoint += b.position;
	b.RShoulderDef.anchorPoint += b.position;
	b.LWristDef.anchorPoint += b.position;
	b.RWristDef.anchorPoint += b.position;

	// create body parts
	LFoot		= w->CreateBody(&b.LFootDef);
	RFoot		= w->CreateBody(&b.RFootDef);
	LCalf		= w->CreateBody(&b.LCalfDef);
	RCalf		= w->CreateBody(&b.RCalfDef);
	LThigh		= w->CreateBody(&b.LThighDef);
	RThigh		= w->CreateBody(&b.RThighDef);
	Pelvis		= w->CreateBody(&b.PelvisDef);
	Stomach		= w->CreateBody(&b.StomachDef);
	Chest		= w->CreateBody(&b.ChestDef);
	Neck		= w->CreateBody(&b.NeckDef);
	Head		= w->CreateBody(&b.HeadDef);
	LUpperArm	= w->CreateBody(&b.LUpperArmDef);
	RUpperArm	= w->CreateBody(&b.RUpperArmDef);
	LForearm	= w->CreateBody(&b.LForearmDef);
	RForearm	= w->CreateBody(&b.RForearmDef);
	LHand		= w->CreateBody(&b.LHandDef);
	RHand		= w->CreateBody(&b.RHandDef);
	
	// link body parts
	b.LAnkleDef.body1		= LFoot;
	b.LAnkleDef.body2		= LCalf;
	b.RAnkleDef.body1		= RFoot;
	b.RAnkleDef.body2		= RCalf;
	b.LKneeDef.body1		= LCalf;
	b.LKneeDef.body2		= LThigh;
	b.RKneeDef.body1		= RCalf;
	b.RKneeDef.body2		= RThigh;
	b.LHipDef.body1			= LThigh;
	b.LHipDef.body2			= Pelvis;
	b.RHipDef.body1			= RThigh;
	b.RHipDef.body2			= Pelvis;
	b.LowerAbsDef.body1		= Pelvis;
	b.LowerAbsDef.body2		= Stomach;
	b.UpperAbsDef.body1		= Stomach;
	b.UpperAbsDef.body2		= Chest;
	b.LowerNeckDef.body1	= Chest;
	b.LowerNeckDef.body2	= Neck;
	b.UpperNeckDef.body1	= Chest;
	b.UpperNeckDef.body2	= Head;
	b.LShoulderDef.body1	= Chest;
	b.LShoulderDef.body2	= LUpperArm;
	b.RShoulderDef.body1	= Chest;
	b.RShoulderDef.body2	= RUpperArm;
	b.LElbowDef.body1		= LForearm;
	b.LElbowDef.body2		= LUpperArm;
	b.RElbowDef.body1		= RForearm;
	b.RElbowDef.body2		= RUpperArm;
	b.LWristDef.body1		= LHand;
	b.LWristDef.body2		= LForearm;
	b.RWristDef.body1		= RHand;
	b.RWristDef.body2		= RForearm;
	
	// create joints
	LAnkle		= (b2RevoluteJoint*)w->CreateJoint(&b.LAnkleDef);
	RAnkle		= (b2RevoluteJoint*)w->CreateJoint(&b.RAnkleDef);
	LKnee		= (b2RevoluteJoint*)w->CreateJoint(&b.LKneeDef);
	RKnee		= (b2RevoluteJoint*)w->CreateJoint(&b.RKneeDef);
	LHip		= (b2RevoluteJoint*)w->CreateJoint(&b.LHipDef);
	RHip		= (b2RevoluteJoint*)w->CreateJoint(&b.RHipDef);
	LowerAbs	= (b2RevoluteJoint*)w->CreateJoint(&b.LowerAbsDef);
	UpperAbs	= (b2RevoluteJoint*)w->CreateJoint(&b.UpperAbsDef);
	LowerNeck	= (b2RevoluteJoint*)w->CreateJoint(&b.LowerNeckDef);
	UpperNeck	= (b2RevoluteJoint*)w->CreateJoint(&b.UpperNeckDef);
	LShoulder	= (b2RevoluteJoint*)w->CreateJoint(&b.LShoulderDef);
	RShoulder	= (b2RevoluteJoint*)w->CreateJoint(&b.RShoulderDef);
	LElbow		= (b2RevoluteJoint*)w->CreateJoint(&b.LElbowDef);
	RElbow		= (b2RevoluteJoint*)w->CreateJoint(&b.RElbowDef);
	LWrist		= (b2RevoluteJoint*)w->CreateJoint(&b.LWristDef);
	RWrist		= (b2RevoluteJoint*)w->CreateJoint(&b.RWristDef);
}


void Biped::SetMotorTorque(float f)
{
	LAnkle->SetMotorTorque(f);
	RAnkle->SetMotorTorque(f);
	LKnee->SetMotorTorque(f);
	RKnee->SetMotorTorque(f);
	LHip->SetMotorTorque(f);
	RHip->SetMotorTorque(f);
	LowerAbs->SetMotorTorque(f);
	UpperAbs->SetMotorTorque(f);
	LowerNeck->SetMotorTorque(f);
	UpperNeck->SetMotorTorque(f);
	LShoulder->SetMotorTorque(f);
	RShoulder->SetMotorTorque(f);
	LElbow->SetMotorTorque(f);
	RElbow->SetMotorTorque(f);
	LWrist->SetMotorTorque(f);
	RWrist->SetMotorTorque(f);
}

float Biped::GetMass()
{
	return	LFoot->GetMass()
		+	RFoot->GetMass()
		+	LCalf->GetMass()
		+	RCalf->GetMass()
		+	LThigh->GetMass()
		+	RThigh->GetMass()
		+	Pelvis->GetMass()
		+	Stomach->GetMass()
		+	Chest->GetMass()
		+	Neck->GetMass()		
		+	Head->GetMass()			
		+	LUpperArm->GetMass()	
		+	RUpperArm->GetMass()	
		+	LForearm->GetMass()		
		+	RForearm->GetMass()		
		+	LHand->GetMass()		
		+	RHand->GetMass();
}

b2Vec2 Biped::GetCenterPosition()
{
	b2Vec2 p = 
			LFoot->GetCenterPosition()
		+	RFoot->GetCenterPosition()
		+	LCalf->GetCenterPosition()
		+	RCalf->GetCenterPosition()
		+	LThigh->GetCenterPosition()
		+	RThigh->GetCenterPosition()
		+	Pelvis->GetCenterPosition()
		+	Stomach->GetCenterPosition()
		+	Chest->GetCenterPosition()
		+	Neck->GetCenterPosition()		
		+	Head->GetCenterPosition()			
		+	LUpperArm->GetCenterPosition()	
		+	RUpperArm->GetCenterPosition()	
		+	LForearm->GetCenterPosition()		
		+	RForearm->GetCenterPosition()		
		+	LHand->GetCenterPosition()		
		+	RHand->GetCenterPosition();
	p.x /= 17; 
	p.y /= 17;
	return p;
}

Biped::~Biped(void)
{
}
