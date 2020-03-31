//
// Copyright (c) 2009, Markus Rickert
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#ifndef RL_SG_DEPTHSCENE_H
#define RL_SG_DEPTHSCENE_H

#include <rl_build/include/math/Vector.h>

#include "Scene.h"

namespace rl
{
	namespace sg
	{
		class Body;
		class Shape;
		
		class DepthScene : public virtual Scene
		{
		public:
			DepthScene();
			
			virtual ~DepthScene();
			
			virtual ::rl::math::Real depth(Body* first, Body* second, ::rl::math::Vector3& point1, ::rl::math::Vector3& point2);
			
			virtual ::rl::math::Real depth(Model* first, Model* second, ::rl::math::Vector3& point1, ::rl::math::Vector3& point2);
			
			virtual ::rl::math::Real depth(Shape* first, Shape* second, ::rl::math::Vector3& point1, ::rl::math::Vector3& point2) = 0;
			
		protected:
			
		private:
			
		};
	}
}

#endif // RL_SG_DEPTHSCENE_H