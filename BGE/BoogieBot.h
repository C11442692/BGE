#pragma once
#include "Game.h"
#include "PhysicsController.h"
#include "PhysicsFactory.h"
#include <btBulletDynamicsCommon.h>

namespace BGE
{
	class BoogieBot :
		public Game
	{
	private:

	public:
		BoogieBot(void);
		~BoogieBot(void);
		bool Initialise();
		void Update();
		void Cleanup();
		void LegPos1();
		void FootPos1();
		void LegPos2();
		void FootPos2();
		int size;
		int height;
		int width;
		btHingeConstraint * frhinge;
		btHingeConstraint * flhinge;
		btHingeConstraint * brhinge;
		btHingeConstraint * blhinge;
		btHingeConstraint * headhinge;

		btHingeConstraint * fhinge1;
		btHingeConstraint * fhinge2;
		btHingeConstraint * fhinge3;
		btHingeConstraint * fhinge4;
		btScalar force;

		float time;
	};
}
