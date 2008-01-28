#include "Biped.h"
#include "BipedDef.h"

Biped::Biped(b2World* w, const b2Vec2& position)				
{
	m_world = w;

	BipedDef def;
	b2BodyDef bd;

	// create body parts
	bd = def.LFootDef;
	bd.position += position;
	LFoot = w->Create(&bd);
	LFoot->AddShape(&def.LFootPoly);
	LFoot->SetMassFromShapes();

	bd = def.RFootDef;
	bd.position += position;
	RFoot = w->Create(&bd);
	RFoot->AddShape(&def.RFootPoly);
	RFoot->SetMassFromShapes();

	bd = def.LCalfDef;
	bd.position += position;
	LCalf = w->Create(&bd);
	LCalf->AddShape(&def.LCalfPoly);
	LCalf->SetMassFromShapes();

	bd = def.RCalfDef;
	bd.position += position;
	RCalf = w->Create(&bd);
	RCalf->AddShape(&def.RCalfPoly);
	RCalf->SetMassFromShapes();

	bd = def.LThighDef;
	bd.position += position;
	LThigh = w->Create(&bd);
	LThigh->AddShape(&def.LThighPoly);
	LThigh->SetMassFromShapes();

	bd = def.RThighDef;
	bd.position += position;
	RThigh = w->Create(&bd);
	RThigh->AddShape(&def.RThighPoly);
	RThigh->SetMassFromShapes();

	bd = def.PelvisDef;
	bd.position += position;
	Pelvis = w->Create(&bd);
	Pelvis->AddShape(&def.PelvisPoly);
	Pelvis->SetMassFromShapes();

	bd = def.PelvisDef;
	bd.position += position;
	Stomach = w->Create(&bd);
	Stomach->AddShape(&def.StomachPoly);
	Stomach->SetMassFromShapes();

	bd = def.ChestDef;
	bd.position += position;
	Chest = w->Create(&bd);
	Chest->AddShape(&def.ChestPoly);
	Chest->SetMassFromShapes();

	bd = def.NeckDef;
	bd.position += position;
	Neck = w->Create(&bd);
	Neck->AddShape(&def.NeckPoly);
	Neck->SetMassFromShapes();

	bd = def.HeadDef;
	bd.position += position;
	Head = w->Create(&bd);
	Head->AddShape(&def.HeadCirc);
	Head->SetMassFromShapes();

	bd = def.LUpperArmDef;
	bd.position += position;
	LUpperArm = w->Create(&bd);
	LUpperArm->AddShape(&def.LUpperArmPoly);
	LUpperArm->SetMassFromShapes();

	bd = def.RUpperArmDef;
	bd.position += position;
	RUpperArm = w->Create(&bd);
	RUpperArm->AddShape(&def.RUpperArmPoly);
	RUpperArm->SetMassFromShapes();

	bd = def.LForearmDef;
	bd.position += position;
	LForearm = w->Create(&bd);
	LForearm->AddShape(&def.LForearmPoly);
	LForearm->SetMassFromShapes();

	bd = def.RForearmDef;
	bd.position += position;
	RForearm = w->Create(&bd);
	RForearm->AddShape(&def.RForearmPoly);
	RForearm->SetMassFromShapes();

	bd = def.LHandDef;
	bd.position += position;
	LHand = w->Create(&bd);
	LHand->AddShape(&def.LHandPoly);
	LHand->SetMassFromShapes();

	bd = def.RHandDef;
	bd.position += position;
	RHand = w->Create(&bd);
	RHand->AddShape(&def.RHandPoly);
	RHand->SetMassFromShapes();
	
	// link body parts
	def.LAnkleDef.body1		= LFoot;
	def.LAnkleDef.body2		= LCalf;
	def.RAnkleDef.body1		= RFoot;
	def.RAnkleDef.body2		= RCalf;
	def.LKneeDef.body1		= LCalf;
	def.LKneeDef.body2		= LThigh;
	def.RKneeDef.body1		= RCalf;
	def.RKneeDef.body2		= RThigh;
	def.LHipDef.body1		= LThigh;
	def.LHipDef.body2		= Pelvis;
	def.RHipDef.body1		= RThigh;
	def.RHipDef.body2		= Pelvis;
	def.LowerAbsDef.body1	= Pelvis;
	def.LowerAbsDef.body2	= Stomach;
	def.UpperAbsDef.body1	= Stomach;
	def.UpperAbsDef.body2	= Chest;
	def.LowerNeckDef.body1	= Chest;
	def.LowerNeckDef.body2	= Neck;
	def.UpperNeckDef.body1	= Chest;
	def.UpperNeckDef.body2	= Head;
	def.LShoulderDef.body1	= Chest;
	def.LShoulderDef.body2	= LUpperArm;
	def.RShoulderDef.body1	= Chest;
	def.RShoulderDef.body2	= RUpperArm;
	def.LElbowDef.body1		= LForearm;
	def.LElbowDef.body2		= LUpperArm;
	def.RElbowDef.body1		= RForearm;
	def.RElbowDef.body2		= RUpperArm;
	def.LWristDef.body1		= LHand;
	def.LWristDef.body2		= LForearm;
	def.RWristDef.body1		= RHand;
	def.RWristDef.body2		= RForearm;
	
	// create joints
	LAnkle		= (b2RevoluteJoint*)w->Create(&def.LAnkleDef);
	RAnkle		= (b2RevoluteJoint*)w->Create(&def.RAnkleDef);
	LKnee		= (b2RevoluteJoint*)w->Create(&def.LKneeDef);
	RKnee		= (b2RevoluteJoint*)w->Create(&def.RKneeDef);
	LHip		= (b2RevoluteJoint*)w->Create(&def.LHipDef);
	RHip		= (b2RevoluteJoint*)w->Create(&def.RHipDef);
	LowerAbs	= (b2RevoluteJoint*)w->Create(&def.LowerAbsDef);
	UpperAbs	= (b2RevoluteJoint*)w->Create(&def.UpperAbsDef);
	LowerNeck	= (b2RevoluteJoint*)w->Create(&def.LowerNeckDef);
	UpperNeck	= (b2RevoluteJoint*)w->Create(&def.UpperNeckDef);
	LShoulder	= (b2RevoluteJoint*)w->Create(&def.LShoulderDef);
	RShoulder	= (b2RevoluteJoint*)w->Create(&def.RShoulderDef);
	LElbow		= (b2RevoluteJoint*)w->Create(&def.LElbowDef);
	RElbow		= (b2RevoluteJoint*)w->Create(&def.RElbowDef);
	LWrist		= (b2RevoluteJoint*)w->Create(&def.LWristDef);
	RWrist		= (b2RevoluteJoint*)w->Create(&def.RWristDef);
}


Biped::~Biped(void)
{
	m_world->Destroy(LFoot);
	m_world->Destroy(RFoot);
	m_world->Destroy(LCalf);
	m_world->Destroy(RCalf);
	m_world->Destroy(LThigh);
	m_world->Destroy(RThigh);
	m_world->Destroy(Pelvis);
	m_world->Destroy(Stomach);
	m_world->Destroy(Chest);
	m_world->Destroy(Neck);
	m_world->Destroy(Head);
	m_world->Destroy(LUpperArm);
	m_world->Destroy(RUpperArm);
	m_world->Destroy(LForearm);
	m_world->Destroy(RForearm);
	m_world->Destroy(LHand);
	m_world->Destroy(RHand);
}
