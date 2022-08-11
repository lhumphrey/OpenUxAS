// ===============================================================================
// Authors: AFRL/RQQA
// Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
// 
// Copyright (c) 2017 Government of the United State of America, as represented by
// the Secretary of the Air Force.  No copyright is claimed in the United States under
// Title 17, U.S. Code.  All Other Rights Reserved.
// ===============================================================================

/* 
 * File:   RouteAggregatorServiceNew.cpp
 * Author: LRH
 * 
 * Created on April 22, 2021, 2:34 PM
 */

#include "RouteAggregatorServiceNew.h"

#include "UxAS_Log.h"
#include "pugixml.hpp"
#include "UnitConversions.h"
#include "DRand.h" //unneeded?
#include "Constants/UxAS_String.h"

#include <map>

#define STRING_COMPONENT_NAME "RouteAggregator"
#define STRING_XML_COMPONENT_TYPE STRING_COMPONENT_NAME
#define STRING_XML_COMPONENT "Component"
#define STRING_XML_TYPE "Type"
#define STRING_XML_FAST_PLAN "FastPlan"

namespace uxas
{
namespace service
{
RouteAggregatorServiceNew::ServiceBase::CreationRegistrar<RouteAggregatorServiceNew>
RouteAggregatorServiceNew::s_registrar(RouteAggregatorServiceNew::s_registryServiceTypeNames());

RouteAggregatorServiceNew::RouteAggregatorServiceNew()
: ServiceBase(RouteAggregatorServiceNew::s_typeName(), RouteAggregatorServiceNew::s_directoryName()) { };

RouteAggregatorServiceNew::~RouteAggregatorServiceNew() { };

bool
RouteAggregatorServiceNew::initialize()
{
    return true;
}

bool
RouteAggregatorServiceNew::configure(const pugi::xml_node& ndComponent)

{
    std::string strBasePath = m_workDirectoryPath;
    uint32_t ui32EntityID = m_entityId;
    uint32_t ui32LmcpMessageSize_max = 100000;
    std::stringstream sstrErrors;

    std::string strComponentType = ndComponent.attribute(STRING_XML_TYPE).value();

    if (!ndComponent.attribute(STRING_XML_FAST_PLAN).empty())
    {
        // Only supported parameter: disables local route planner for
        // computationally expensive ground route calculations
        m_fastPlan = ndComponent.attribute(STRING_XML_FAST_PLAN).as_bool();
    }

    // Track states and configurations for assignment cost matrix calculation
    // [EntityStates] are used to calculate costs from current position to first task
    // [EntityConfigurations] are used for nominal speed values (all costs are in terms of time to arrive)
    
    //ENTITY CONFIGURATIONS
    addSubscriptionAddress(afrl::cmasi::EntityConfiguration::Subscription);
    std::vector< std::string > childconfigs = afrl::cmasi::EntityConfigurationDescendants();
    for(auto child : childconfigs)
        addSubscriptionAddress(child);
    
    // ENTITY STATES
    addSubscriptionAddress(afrl::cmasi::EntityState::Subscription);
    std::vector< std::string > childstates = afrl::cmasi::EntityStateDescendants();
    for(auto child : childstates)
        addSubscriptionAddress(child);

    // UniqueAutomationRequest messages kickoff matrix calculation
    addSubscriptionAddress(uxas::messages::task::UniqueAutomationRequest::Subscription);

    // subscribe to task plan options to build matrix
    addSubscriptionAddress(uxas::messages::task::TaskPlanOptions::Subscription);

    // handle batch route requests
    addSubscriptionAddress(uxas::messages::route::RouteRequest::Subscription);

    // listen for responses to requests from route planner(s)
    addSubscriptionAddress(uxas::messages::route::RoutePlanResponse::Subscription);

    // Subscribe to group messages (whisper from local route planner)
    //TODO REVIEW DESIGN "RouteAggregator" "RoutePlanner" flip message addressing effecting session behavior

    return true; // may not have the proper fast plan value, but proceed anyway
}

bool
RouteAggregatorServiceNew::processReceivedLmcpMessage(std::unique_ptr<uxas::communications::data::LmcpMessage> receivedLmcpMessage)
{
    if (uxas::messages::route::isRoutePlanResponse(receivedLmcpMessage->m_object.get()))
    {
        auto routePlanResp = std::static_pointer_cast<uxas::messages::route::RoutePlanResponse>(receivedLmcpMessage->m_object);
        m_routePlanResponses[routePlanResp->getResponseID()] = routePlanResp;      
        CheckAllRoutePlans();
    }
    else if (uxas::messages::route::isRouteRequest(receivedLmcpMessage->m_object.get()))
    {
        auto routeReq = std::static_pointer_cast<uxas::messages::route::RouteRequest>(receivedLmcpMessage->m_object);
        HandleRouteRequest(routeReq);
    }
    else if (std::dynamic_pointer_cast<afrl::cmasi::AirVehicleState>(receivedLmcpMessage->m_object))
    {
        int64_t id = std::static_pointer_cast<afrl::cmasi::EntityState>(receivedLmcpMessage->m_object)->getID();
        m_entityStates[id] = std::static_pointer_cast<afrl::cmasi::EntityState>(receivedLmcpMessage->m_object);
        m_airVehicles.insert(id);
    }
    else if (afrl::vehicles::isGroundVehicleState(receivedLmcpMessage->m_object.get()))
    {
        int64_t id = std::static_pointer_cast<afrl::cmasi::EntityState>(receivedLmcpMessage->m_object)->getID();
        m_entityStates[id] = std::static_pointer_cast<afrl::cmasi::EntityState>(receivedLmcpMessage->m_object);
        m_groundVehicles.insert(id);
    }
    else if (afrl::vehicles::isSurfaceVehicleState(receivedLmcpMessage->m_object.get()))
    {
        int64_t id = std::static_pointer_cast<afrl::cmasi::EntityState>(receivedLmcpMessage->m_object)->getID();
        m_entityStates[id] = std::static_pointer_cast<afrl::cmasi::EntityState>(receivedLmcpMessage->m_object);
        m_surfaceVehicles.insert(id);
    }
    else if (std::dynamic_pointer_cast<afrl::cmasi::AirVehicleConfiguration>(receivedLmcpMessage->m_object))
    {
        int64_t id = std::static_pointer_cast<afrl::cmasi::EntityConfiguration>(receivedLmcpMessage->m_object)->getID();
        m_entityConfigurations[id] = std::static_pointer_cast<afrl::cmasi::EntityConfiguration>(receivedLmcpMessage->m_object);
        m_airVehicles.insert(id);
    }
    else if (afrl::vehicles::isGroundVehicleConfiguration(receivedLmcpMessage->m_object.get()))
    {
        int64_t id = std::static_pointer_cast<afrl::cmasi::EntityConfiguration>(receivedLmcpMessage->m_object)->getID();
        m_entityConfigurations[id] = std::static_pointer_cast<afrl::cmasi::EntityConfiguration>(receivedLmcpMessage->m_object);
        m_groundVehicles.insert(id);
    }
    else if (afrl::vehicles::isSurfaceVehicleConfiguration(receivedLmcpMessage->m_object.get()))
    {
        int64_t id = std::static_pointer_cast<afrl::cmasi::EntityConfiguration>(receivedLmcpMessage->m_object)->getID();
        m_entityConfigurations[id] = std::static_pointer_cast<afrl::cmasi::EntityConfiguration>(receivedLmcpMessage->m_object);
        m_surfaceVehicles.insert(id);
    }
    else if (uxas::messages::task::isUniqueAutomationRequest(receivedLmcpMessage->m_object.get()))
    {
        auto uaReq = std::static_pointer_cast<uxas::messages::task::UniqueAutomationRequest>(receivedLmcpMessage->m_object);
        m_uniqueAutomationRequests[uaReq->getRequestID()] = uaReq;
        CheckAllTaskOptionsReceived();
    }
    else if (afrl::impact::isImpactAutomationRequest(receivedLmcpMessage->m_object.get()))
    {
        auto iaReq = std::static_pointer_cast<afrl::impact::ImpactAutomationRequest>(receivedLmcpMessage->m_object);
        auto uaReq = std::shared_ptr<uxas::messages::task::UniqueAutomationRequest>();
        uaReq->setOriginalRequest(iaReq->getTrialRequest()->clone());
        m_uniqueAutomationRequests[uaReq->getRequestID()] = uaReq;
        CheckAllTaskOptionsReceived();
    }
    else if (uxas::messages::task::isTaskPlanOptions(receivedLmcpMessage->m_object.get()))
    {
        auto taskPlanOpts = std::static_pointer_cast<uxas::messages::task::TaskPlanOptions>(receivedLmcpMessage->m_object);
        m_taskPlanOptions[taskPlanOpts->getTaskID()] = taskPlanOpts;
        CheckAllTaskOptionsReceived();
    }
    return (false); // always false implies never terminating service from here
}

void RouteAggregatorServiceNew::CheckAllTaskOptionsReceived()
{
    // For each UniqueAutomationRequest, check whether all necessary TaskPlanOptions for it
    // have been received. If so, call BuildMatrixRequests to generate RoutePlanRequests
    auto uaReqIter = m_uniqueAutomationRequests.begin();
    while (uaReqIter != m_uniqueAutomationRequests.end())
    {
        bool isAllReceived{true};
        for (size_t t = 0; t < uaReqIter->second->getOriginalRequest()->getTaskList().size(); t++)
        {
            int64_t taskId = uaReqIter->second->getOriginalRequest()->getTaskList().at(t);
            if (m_taskPlanOptions.find(taskId) == m_taskPlanOptions.end())
            {
                isAllReceived = false;
                break;
            }
        }

        if (isAllReceived)
        {
            BuildMatrixRequests(uaReqIter->first, uaReqIter->second);

        }
        uaReqIter++;
    }
}

void RouteAggregatorServiceNew::BuildMatrixRequests (int64_t uaReqId, const std::shared_ptr<uxas::messages::task::UniqueAutomationRequest>& uaReq)
{
    // Precondition: All TaskPlanOptions corresponding to a UniqueAutomationRequest have been received. 
    // 
    // Goal: Send a set of RoutePlanRequest messages to get possible routes to fulfill the UniqueAutonomationRequest.

    // Procedure:
    //  1. For each vehicle, create a RoutePlanRequest with a unique value for RoutePlanRequest.RequestID and 
    //     and field RouteRequests : RouteConstraints[] such that each RouteConstraint.RouteID is unique and 
    //     is associated with task options in m_routeTaskPairing. The RouteConstraint list should contain the 
    //     following for each task for which the vehicle is eligible
    //       A. Initial position of the vehicle to the start of each task option
    //       B. For pairs tasks T1 and T2, end of each T1 option to start of each T2 option
    //  2. Record each RoutePlanRequest.RequestID for the UniqueAutonomtionRequest.RequestID in 
    //     m_uaReqIdToPendingRoutePlanResponseIds (to be used to check whether all corresponding 
    //     RoutePlanResponse messages have been received)
    //  3. Send requests to proper planners

    m_uniqueAutomationRequestIdToPendingRoutePlanResponseIds[uaReqId] = std::unordered_set<int64_t>();
    std::vector< std::shared_ptr<uxas::messages::route::RoutePlanRequest> > sendAirPlanRequest;
    std::vector< std::shared_ptr<uxas::messages::route::RoutePlanRequest> > sendGroundPlanRequest;
    
    // If the EntityList is empty, then ALL vehicles are considered eligible
    if(uaReq->getOriginalRequest()->getEntityList().empty())
    {
        for(auto entity : m_entityStates)
        {
            uaReq->getOriginalRequest()->getEntityList().push_back(entity.second->getID());
        }
    }

    // Make RoutePlanRequest for each vehicle
    for (size_t v = 0; v < uaReq->getOriginalRequest()->getEntityList().size(); v++)
    {
        // Only use vehicles that have a valid EntityState or UniqueAutomationRequest.PlanningState
        int64_t vehicleId = uaReq->getOriginalRequest()->getEntityList().at(v);
        auto vehicleStateCursor = m_entityStates.find(vehicleId);

        float startHeading_deg{0.0};
        auto startLocation = std::shared_ptr<afrl::cmasi::Location3D>();
        bool isFoundPlannningState{false};
        for (auto& planningState : uaReq->getPlanningStates())
        {
            if (planningState->getEntityID() == vehicleId)
            {
                startLocation.reset(planningState->getPlanningPosition()->clone());
                startHeading_deg = planningState->getPlanningHeading();
                isFoundPlannningState = true;
                break;
            }
        }

        if (isFoundPlannningState || (vehicleStateCursor != m_entityStates.end()))
        {
            // Build list of task options this vehicle is eligible for
            std::vector<std::shared_ptr<uxas::messages::task::TaskOption> > taskOptionList;
            for (size_t t = 0; t < uaReq->getOriginalRequest()->getTaskList().size(); t++)
            {
                int64_t taskId = uaReq->getOriginalRequest()->getTaskList().at(t);
                if (m_taskPlanOptions.find(taskId) != m_taskPlanOptions.end())
                {
                    for (size_t o = 0; o < m_taskPlanOptions[taskId]->getOptions().size(); o++)
                    {
                        auto option = m_taskPlanOptions[taskId]->getOptions().at(o);

                        auto elig = std::find_if(option->getEligibleEntities().begin(), option->getEligibleEntities().end(),
                                                 [&](int64_t v)
                                                 {
                                                     return v == vehicleId; });
                        if (option->getEligibleEntities().empty() || elig != option->getEligibleEntities().end())
                        {
                            taskOptionList.push_back(std::shared_ptr<uxas::messages::task::TaskOption>(option->clone()));
                        }
                    }
                }
            }

            // Create a new RoutePlanRequest
            std::shared_ptr<uxas::messages::route::RoutePlanRequest> routePlanReq(new uxas::messages::route::RoutePlanRequest);
            routePlanReq->setAssociatedTaskID(0); // mapping from routeID to proper task
            routePlanReq->setIsCostOnlyRequest(false);  // request full path for more accurate timing information
            routePlanReq->setOperatingRegion(uaReq->getOriginalRequest()->getOperatingRegion());
            routePlanReq->setVehicleID(vehicleId);
            routePlanReq->setRequestID(m_routePlanRequestId);
            m_uniqueAutomationRequestIdToPendingRoutePlanResponseIds[uaReq->getRequestID()].insert(m_routePlanRequestId);
            m_routePlanRequestId++;

            if (!isFoundPlannningState)
            {
                assert(vehicleStateCursor != m_entityStates.end());
                startLocation.reset(vehicleStateCursor->second->getLocation()->clone());
                startHeading_deg = vehicleStateCursor->second->getHeading();
            }

            // Create RouteConstraint from vehicle initial position to each task option
            for (size_t t = 0; t < taskOptionList.size(); t++)
            {
                auto option = taskOptionList.at(t);

                // build map from request to full task/option information
                AggregatorTaskOptionPair* top = new AggregatorTaskOptionPair(vehicleId, 0, 0, option->getTaskID(), option->getOptionID());
                m_routeTaskPairing[m_routeId] = std::shared_ptr<AggregatorTaskOptionPair>(top);

                uxas::messages::route::RouteConstraints* rc = new uxas::messages::route::RouteConstraints;
                rc->setStartLocation(startLocation->clone());
                rc->setStartHeading(startHeading_deg);
                rc->setEndLocation(option->getStartLocation()->clone());
                rc->setEndHeading(option->getStartHeading());
                rc->setRouteID(m_routeId);
                m_routeId++;
                routePlanReq->getRouteRequests().push_back(rc);
            }

            // Create RouteConstraint for each combition of options of different tasks
            for (size_t t1 = 0; t1 < taskOptionList.size(); t1++)
            {
                for (size_t t2 = 0; t2 < taskOptionList.size(); t2++)
                {
                    auto option1 = taskOptionList.at(t1);
                    auto option2 = taskOptionList.at(t2);

                    if (t1 != t2 && option1->getTaskID() != option2->getTaskID())
                    {
                        // Build map from request to full task/option information
                        AggregatorTaskOptionPair* top = new AggregatorTaskOptionPair(vehicleId, option1->getTaskID(), option1->getOptionID(), option2->getTaskID(), option2->getOptionID());
                        m_routeTaskPairing[m_routeId] = std::shared_ptr<AggregatorTaskOptionPair>(top);

                        uxas::messages::route::RouteConstraints* rc = new uxas::messages::route::RouteConstraints;
                        rc->setStartLocation(option1->getEndLocation()->clone());
                        rc->setStartHeading(option1->getEndHeading());
                        rc->setEndLocation(option2->getStartLocation()->clone());
                        rc->setEndHeading(option2->getStartHeading());
                        rc->setRouteID(m_routeId);
                        routePlanReq->getRouteRequests().push_back(rc);
                        m_routeId++;
                    }
                }
            }

            // send this plan request to the prescribed route planner for ground vehicles
            if (m_groundVehicles.find(vehicleId) != m_groundVehicles.end())
            {
                sendGroundPlanRequest.push_back(routePlanReq);
            }
            else
            {
                sendAirPlanRequest.push_back(routePlanReq);
            }
        }
    }

    // Send all requests for aircraft plans
    for (size_t k = 0; k < sendAirPlanRequest.size(); k++)
    {
        std::shared_ptr<avtas::lmcp::Object> pRequest = std::static_pointer_cast<avtas::lmcp::Object>(sendAirPlanRequest.at(k));
        sendSharedLmcpObjectLimitedCastMessage(uxas::common::MessageGroup::AircraftPathPlanner(), pRequest);
    }

    // Send all requests for ground plans
    for (size_t k = 0; k < sendGroundPlanRequest.size(); k++)
    {
        std::shared_ptr<avtas::lmcp::Object> pRequest = std::static_pointer_cast<avtas::lmcp::Object>(sendGroundPlanRequest.at(k));
        if (m_fastPlan)
        {
            // short-circuit and just plan with straight line planner
            EuclideanPlan(sendGroundPlanRequest.at(k));
        }
        else
        {
            // send externally
            sendSharedLmcpObjectLimitedCastMessage(uxas::common::MessageGroup::GroundPathPlanner(), pRequest);
        }
    }

    // fast planning should be complete, so kick off sending response
    if (m_fastPlan)
    {
        CheckAllRoutePlans();
    }
}

void RouteAggregatorServiceNew::HandleRouteRequest(std::shared_ptr<uxas::messages::route::RouteRequest> routeReq)
{
    // If no vehicles are specified, all vehicles are eligible
    if (routeReq->getVehicleID().empty())
    {
        routeReq->getVehicleID().reserve(m_entityStates.size());
        for (const auto& v : m_entityStates)
        {
            routeReq->getVehicleID().push_back(v.second->getID());
        }
    }

    // Generate a RoutePlanRequest for each eligible vehicle
    for (const int64_t& vehicleId : routeReq->getVehicleID())
    {
        std::shared_ptr<uxas::messages::route::RoutePlanRequest> routePlanReq(new uxas::messages::route::RoutePlanRequest);
        routePlanReq->setRequestID(m_routePlanRequestId);
        routePlanReq->setAssociatedTaskID(routeReq->getAssociatedTaskID());
        routePlanReq->setIsCostOnlyRequest(routeReq->getIsCostOnlyRequest());
        routePlanReq->setOperatingRegion(routeReq->getOperatingRegion());
        routePlanReq->setVehicleID(vehicleId);
        for (auto& r : routeReq->getRouteRequests())
        {
            routePlanReq->getRouteRequests().push_back(r->clone());
        }

        // Map this RouteRequest.RequestID to generated RoutePlanRequest.RequestID
        m_routeRequestIdToPendingRoutePlanResponseIds[routeReq->getRequestID()].insert(m_routePlanRequestId);

        // Increment so that each RoutePlanRequest.RequestID will be unique
        m_routePlanRequestId++;

        std::shared_ptr<avtas::lmcp::Object> pRequest = std::static_pointer_cast<avtas::lmcp::Object>(routePlanReq);
        if (m_groundVehicles.find(vehicleId) != m_groundVehicles.end())
        {
            if (m_fastPlan)
            {
                // short-circuit and just plan with straight line planner
                EuclideanPlan(routePlanReq);
            }
            else
            {
                // send externally
                sendSharedLmcpObjectLimitedCastMessage(uxas::common::MessageGroup::GroundPathPlanner(), pRequest);
            }
        }
        else
        {
            // send to aircraft planner
            sendSharedLmcpObjectLimitedCastMessage(uxas::common::MessageGroup::AircraftPathPlanner(), pRequest);
        }
    }

    // if fast planning, then all routes should be complete; kick off response
    if (m_fastPlan)
    {
        CheckAllRoutePlans();
    }
}

void RouteAggregatorServiceNew::CheckAllRoutePlans()
{
    // Check whether any RouteRequest have all necessary RoutePlanResponse
    auto i = m_routeRequestIdToPendingRoutePlanResponseIds.begin();
    while (i != m_routeRequestIdToPendingRoutePlanResponseIds.end())
    {
        bool isFulfilled = true;
        for (const int64_t& j : i->second)
        {
            if (m_routePlanResponses.find(j) == m_routePlanResponses.end())
            {
                isFulfilled = false;
                break;
            }
        }

        if (isFulfilled)
        {
            SendRouteResponse(i->first);
            i = m_routeRequestIdToPendingRoutePlanResponseIds.erase(i);
        }
        else
        {
            i++;
        }
    }

    // Check whether any UniqueAutomationRequest have all necessary RoutePlanResponse
    auto k = m_uniqueAutomationRequestIdToPendingRoutePlanResponseIds.begin();
    while (k != m_uniqueAutomationRequestIdToPendingRoutePlanResponseIds.end())
    {
        bool isFulfilled = true;
        for (const int64_t& j : k->second)
        {
            if (m_routePlanResponses.find(j) == m_routePlanResponses.end())
            {
                isFulfilled = false;
                std::cout << "Not fulfilled" << std::endl;
                break;
            }
        }

        if (isFulfilled)
        {
            std::cout << "Fulfilled" << std::endl;
            SendMatrix(k->first);
            m_uniqueAutomationRequests.erase(k->first);
            k = m_uniqueAutomationRequestIdToPendingRoutePlanResponseIds.erase(k);
        }
        else
        {
            k++;
        }
    }
}

void RouteAggregatorServiceNew::SendRouteResponse(int64_t requestID)
{
    auto routeResp = std::shared_ptr<uxas::messages::route::RouteResponse>(new uxas::messages::route::RouteResponse);
    routeResp->setResponseID(requestID);
    routeResp->getRoutes().reserve(m_routeRequestIdToPendingRoutePlanResponseIds[requestID].size());
    for (auto& rId : m_routeRequestIdToPendingRoutePlanResponseIds[requestID])
    {
        auto routePlanResp = m_routePlanResponses.find(rId);
        if (routePlanResp != m_routePlanResponses.end())
        {
            routeResp->getRoutes().push_back(routePlanResp->second->clone());
            m_routePlanResponses.erase(routePlanResp);
        }
    }

    // Send result
    std::shared_ptr<avtas::lmcp::Object> pResponse = std::static_pointer_cast<avtas::lmcp::Object>(routeResp);
    sendSharedLmcpObjectBroadcastMessage(pResponse);
}

void RouteAggregatorServiceNew::SendMatrix(int64_t uaRequestId)
{
    auto matrix = std::shared_ptr<uxas::messages::task::AssignmentCostMatrix>(new uxas::messages::task::AssignmentCostMatrix);
    auto& uaReq = m_uniqueAutomationRequests[uaRequestId];
    matrix->setCorrespondingAutomationRequestID(uaReq->getRequestID());
    matrix->setOperatingRegion(uaReq->getOriginalRequest()->getOperatingRegion());
    matrix->setTaskLevelRelationship(uaReq->getOriginalRequest()->getTaskRelationships());
    matrix->getTaskList().assign(uaReq->getOriginalRequest()->getTaskList().begin(), uaReq->getOriginalRequest()->getTaskList().end());

    std::stringstream routesNotFound;
    for (auto& routePlanRespId : m_uniqueAutomationRequestIdToPendingRoutePlanResponseIds[uaRequestId])
    {
        auto routePlanResp = m_routePlanResponses[routePlanRespId];
        std::cout << routePlanResp->toString() << std::endl;
        for (auto routePlan : routePlanResp->getRouteResponses())
        {
            std::cout << routePlan->toString() << std::endl;
            auto taskpair = m_routeTaskPairing.find(routePlan->getRouteID());
            std::cout << std::to_string(routePlan->getRouteID()) + " " << taskpair->second->toString() << std::endl;
            if (taskpair != m_routeTaskPairing.end())
            {
                if (routePlan->getRouteCost() < 0)
                {
                    routesNotFound << "V[" << taskpair->second->vehicleId << "](" << taskpair->second->prevTaskId << "," << taskpair->second->prevTaskOption << ")-(" << taskpair->second->taskId << "," << taskpair->second->taskOption << ")" << std::endl;
                }
                auto toc = new uxas::messages::task::TaskOptionCost;
                toc->setDestinationTaskID(taskpair->second->taskId);
                toc->setDestinationTaskOption(taskpair->second->taskOption);
                toc->setIntialTaskID(taskpair->second->prevTaskId);
                toc->setIntialTaskOption(taskpair->second->prevTaskOption);
                toc->setTimeToGo(routePlan->getRouteCost());
                toc->setVehicleID(taskpair->second->vehicleId);
                matrix->getCostMatrix().push_back(toc);
                m_routeTaskPairing.erase(taskpair);
            }
            m_routePlanResponses.erase(routePlanRespId);
        }
    }

    std::cout << matrix->toString() << std::endl;
    for (auto toc : matrix->getCostMatrix()) {
        std::cout << toc->toString() << std::endl;
    }

    // send the total cost matrix
    std::shared_ptr<avtas::lmcp::Object> pResponse = std::static_pointer_cast<avtas::lmcp::Object>(matrix);
    sendSharedLmcpObjectBroadcastMessage(pResponse);

    // clear out old options
    // LRH: Is this assuming only one UAR at a time? I thought the documentation said 
    // you didn't have to assume that
    m_taskPlanOptions.clear();

    if (!routesNotFound.str().empty())
    {
        auto serviceStatus = std::make_shared<afrl::cmasi::ServiceStatus>();
        serviceStatus->setStatusType(afrl::cmasi::ServiceStatusType::Information);
        auto keyValuePair = new afrl::cmasi::KeyValuePair;
        keyValuePair->setKey(std::string("RoutesNotFound - [VehicleId](StartTaskId,StartOptionId)-(EndTaskId,EndOptionId)"));
        keyValuePair->setValue(routesNotFound.str());
        serviceStatus->getInfo().push_back(keyValuePair);
        keyValuePair = nullptr;
        sendSharedLmcpObjectBroadcastMessage(serviceStatus);
        std::cout << "RoutesNotFound - [VehicleId](StartTaskId,StartOptionId)-(EndTaskId,EndOptionId) :: " << std::endl << routesNotFound.str() << std::endl << std::endl;
    }
    else
    {
        auto serviceStatus = std::make_shared<afrl::cmasi::ServiceStatus>();
        serviceStatus->setStatusType(afrl::cmasi::ServiceStatusType::Information);
        auto keyValuePair = new afrl::cmasi::KeyValuePair;
        keyValuePair->setKey(std::string("AssignmentMatrix - full"));
        serviceStatus->getInfo().push_back(keyValuePair);
        keyValuePair = nullptr;
        sendSharedLmcpObjectBroadcastMessage(serviceStatus);
    }

}

void RouteAggregatorServiceNew::EuclideanPlan(std::shared_ptr<uxas::messages::route::RoutePlanRequest> request)
{
    uxas::common::utilities::CUnitConversions flatEarth;
    int64_t regionId = request->getOperatingRegion();
    int64_t vehicleId = request->getVehicleID();
    int64_t taskId = request->getAssociatedTaskID();

    double speed = 1.0; // default if no speed available
    if (m_entityConfigurations.find(vehicleId) != m_entityConfigurations.end())
    {
        double speed = m_entityConfigurations[vehicleId]->getNominalSpeed();
        if (speed < 1e-2)
        {
            speed = 1.0; // default to 1 if too small for division
        }
    }

    auto response = std::shared_ptr<uxas::messages::route::RoutePlanResponse>(new uxas::messages::route::RoutePlanResponse);
    response->setAssociatedTaskID(taskId);
    response->setOperatingRegion(regionId);
    response->setVehicleID(vehicleId);
    response->setResponseID(request->getRequestID());

    for (size_t k = 0; k < request->getRouteRequests().size(); k++)
    {
        uxas::messages::route::RouteConstraints* routeRequest = request->getRouteRequests().at(k);
        int64_t routeId = routeRequest->getRouteID();
        VisiLibity::Point startPt, endPt;
        double north, east;

        uxas::messages::route::RoutePlan* plan = new uxas::messages::route::RoutePlan;
        plan->setRouteID(routeId);

        flatEarth.ConvertLatLong_degToNorthEast_m(routeRequest->getStartLocation()->getLatitude(), routeRequest->getStartLocation()->getLongitude(), north, east);
        startPt.set_x(east);
        startPt.set_y(north);

        flatEarth.ConvertLatLong_degToNorthEast_m(routeRequest->getEndLocation()->getLatitude(), routeRequest->getEndLocation()->getLongitude(), north, east);
        endPt.set_x(east);
        endPt.set_y(north);

        double linedist = VisiLibity::distance(startPt, endPt);
        plan->setRouteCost(linedist / speed * 1000); // milliseconds to arrive
        //m_routePlans[routeId] = std::make_pair(request->getRequestID(), std::shared_ptr<uxas::messages::route::RoutePlan>(plan));
    }
    m_routePlanResponses[response->getResponseID()] = response;
}
}; //namespace service
}; //namespace uxas
