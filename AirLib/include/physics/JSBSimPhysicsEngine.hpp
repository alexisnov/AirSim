// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef airsim_core_JSBSimPhysicsEngine_hpp
#define airsim_core_JSBSimPhysicsEngine_hpp

#include "common/Common.hpp"
#include "physics/PhysicsEngineBase.hpp"
#include <iostream>
#include <sstream>
#include <fstream>
#include <memory>
#include "common/CommonStructs.hpp"
#include "common/SteppableClock.hpp"
#include <cinttypes>

namespace msr
{
namespace airlib
{

    class JSBSimPhysicsEngine : public PhysicsEngineBase
    {
    public:
        JSBSimPhysicsEngine()
        {
        }

        //*** Start: UpdatableState implementation ***//
        virtual void resetImplementation() override
        {
        }

        virtual void update() override
        {
            PhysicsEngineBase::update();

            for (PhysicsBody* body_ptr : *this) {
                body_ptr->update();
            }
        }
        virtual void reportState(StateReporter& reporter) override
        {
            for (PhysicsBody* body_ptr : *this) {
                reporter.writeValue("JSBSimPhysicsEngine", true);
                reporter.writeValue("Is Grounded", body_ptr->isGrounded());
            }
            //call base
            UpdatableObject::reportState(reporter);
        }
        //*** End: UpdatableState implementation ***//
    };

} //namespace
} //namespace
#endif
