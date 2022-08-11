// ===============================================================================
// Authors: AFRL/RQQA
// Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
// 
// Copyright (c) 2017 Government of the United State of America, as represented by
// the Secretary of the Air Force.  No copyright is claimed in the United States under
// Title 17, U.S. Code.  All Other Rights Reserved.
// ===============================================================================

/* 
 * File:   RouteAggregatorServiceNew.h
 * Author: lrh
 *
 * Created on April 22, 2021, 2:34 PM
 */

#ifndef UXAS_SERVICE_ROUTE_AGGREGATOR_NEW_SERVICE_H
#define UXAS_SERVICE_ROUTE_AGGREGATOR_NEW_SERVICE_H

#include "ServiceBase.h"
#include "afrl/cmasi/CMASI.h"
#include "afrl/impact/IMPACT.h"
#include "afrl/vehicles/VEHICLES.h"
#include "uxas/messages/route/ROUTE.h"
#include "uxas/messages/task/UXTASK.h"

#include "visilibity.h"

#include <memory>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <cstdint>
#include <string>

namespace uxas
{
namespace service
{
// description of a particular (task+option) to (task+option) 

class AggregatorTaskOptionPair
{
public:

    AggregatorTaskOptionPair() { };

    AggregatorTaskOptionPair(int64_t v, int64_t pt, int64_t pto, int64_t t, int64_t to) {
        vehicleId = v;
        prevTaskId = pt;
        prevTaskOption = pto;
        taskId = t;
        taskOption = to;
    };

    std::string toString() {
        return  "[vehicleId: " + std::to_string(vehicleId) + 
                ", prevTaskId: " + std::to_string(prevTaskId) + 
                ", prevTaskOption: " + std::to_string(prevTaskOption) + 
                ", taskId: " + std::to_string(taskId) + 
                ", taskOption: " + std::to_string(taskOption);
    }

    ~AggregatorTaskOptionPair() { };

    int64_t vehicleId{0};
    int64_t taskId{0};
    int64_t taskOption{0};
    int64_t prevTaskId{0};
    int64_t prevTaskOption{0};
};


/*! \class RouteAggregatorServiceNew
    \brief A component that incrementally queries the route planner to build
 *   a matrix of plans between all tasks and entity initial points 

 * 
 * Configuration String: 
 *  <Service Type="RouteAggregatorServiceNew" FastPlan="FALSE" />
 * 
 * Options:
 *  - FastPlan
 * 
 * Subscribed Messages:
 *  - afrl::cmasi::AirVehicleState
 *  - afrl::vehicles::GroundVehicleState
 *  - afrl::vehicles::SurfaceVehicleState
 *  - afrl::cmasi::AirVehicleConfiguration
 *  - afrl::vehicles::GroundVehicleConfiguration
 *  - afrl::vehicles::SurfaceVehicleConfiguration
 *  - uxas::messages::task::UniqueAutomationRequest
 *  - uxas::messages::task::TaskPlanOptions
 *  - uxas::messages::route::RouteRequest
 *  - uxas::messages::route::RoutePlanResponse
 * 
 * Sent Messages:
 *  - uxas::messages::route::RoutePlanRequest
 *  - GroundPathPlanner
 *  - AircraftPathPlanner
 *  - uxas::messages::route::RouteResponse
 *  - uxas::messages::task::AssignmentCostMatrix
 *  - afrl::cmasi::ServiceStatus
 * 
 */

class RouteAggregatorServiceNew : public ServiceBase
{
public:

    static const std::string&
    s_typeName() {
        static std::string s_string("RouteAggregatorServiceNew");
        return (s_string);
    };

    static const std::vector<std::string>
    s_registryServiceTypeNames()
    {
        std::vector<std::string> registryServiceTypeNames = {s_typeName()};
        return (registryServiceTypeNames);
    };
    
    static const std::string&
    s_directoryName() {
        static std::string s_string("");
        return (s_string);
    };

    static ServiceBase*
    create() {
        return new RouteAggregatorServiceNew;
    };

    RouteAggregatorServiceNew();

    virtual
    ~RouteAggregatorServiceNew();

private:

    static
    ServiceBase::CreationRegistrar<RouteAggregatorServiceNew> s_registrar;

    /** brief Copy construction not permitted */
    RouteAggregatorServiceNew(RouteAggregatorServiceNew const&) = delete;

    /** brief Copy assignment operation not permitted */
    void operator=(RouteAggregatorServiceNew const&) = delete;

    bool
    configure(const pugi::xml_node& serviceXmlNode) override;

    bool
    initialize() override;

    //bool
    //start() override;

    //bool
    //terminate() override;

    bool
    processReceivedLmcpMessage(std::unique_ptr<uxas::communications::data::LmcpMessage> receivedLmcpMessage) override;


public:


public: //virtual


private:

    void HandleRouteRequest(std::shared_ptr<uxas::messages::route::RouteRequest>);
    void EuclideanPlan(std::shared_ptr<uxas::messages::route::RoutePlanRequest>);
    void CheckAllTaskOptionsReceived();
    void CheckAllRoutePlans();
    void BuildMatrixRequests(int64_t, const std::shared_ptr<uxas::messages::task::UniqueAutomationRequest>&);
    void SendRouteResponse(int64_t);
    void SendMatrix(int64_t);

    // Configurable parameter that disables potentially costly ground route calculations
    // Parameter is identified as 'FastPlan' in the configuration file
    // Fast planning ignores all environment and dynamic constraints and plans straight line only
    bool m_fastPlan{false};

    // Store different types of vehicle state and configuration information
    std::unordered_map<int64_t, std::shared_ptr<afrl::cmasi::EntityState> > m_entityStates;
    std::unordered_map<int64_t, std::shared_ptr<afrl::cmasi::EntityConfiguration> > m_entityConfigurations;
    std::unordered_set<int64_t> m_airVehicles;
    std::unordered_set<int64_t> m_groundVehicles;
    std::unordered_set<int64_t> m_surfaceVehicles;

    // When a UniqueAutomationRequest is received, store it in this map by its RequestID.
    // After an AssignmentCostMatrix is published for it, remove it from the map.
    // Map <UniqueAutomationRequest.RequestID, UniqueAutomationRequest>
    std::unordered_map<int64_t, std::shared_ptr<uxas::messages::task::UniqueAutomationRequest> > m_uniqueAutomationRequests;

    // When a TaskPlanOptions is received from a task, store it in this map by its TaskID.
    // Map <TaskPlanOptions.TaskID, TaskPlanOptions>
    std::unordered_map<int64_t, std::shared_ptr<uxas::messages::task::TaskPlanOptions> > m_taskPlanOptions;

    // Starting ID for uniquely numbering generated RouteConstraints within RoutePlanRequest messages
    int64_t m_routeId{1000000}; // start outside of any task or waypoint id
    // For a list of RoutePlanRequest.RouteQuests : RouteConstraints[], RoutePlanner services
    // should return a list RoutePlanResponse.RouteResponses : RoutePlan[] of the same length where
    // each RoutePlanRequest.RouteQuests[i].RouteID = RoutePlanResponse.RouteResponses[i].RouteID
    
    // Once all the TaskPlanOptions for Tasks referenced in a UniqueAutomationRequest are received, 
    // we can create RoutePlanRequest messages for each vehicle eligible to perform the tasks. 
    // This map is used to store the IDs of all RoutePlanRequest messages that should be 
    // received in response, where RoutePlanRequest.RequestID = RoutePlanResponse.ResponseID.
    // Once all RoutePlanResponse messages have been received, we can create an AssignmentCostMatrix.
    // Map <UniqueAutomationRequest.RequestID, RoutePlanResponse.ResponseID>
    std::unordered_map<int64_t, std::unordered_set<int64_t> > m_uniqueAutomationRequestIdToPendingRoutePlanResponseIds;

    // Mapping from route ID to the corresponding task/option pair
    //         (RoutePlan) route id,      task+option pair
    // LRH not sure whether I need this now. List order should be the same as in RoutePlanResponse
    std::unordered_map<int64_t, std::shared_ptr<AggregatorTaskOptionPair> > m_routeTaskPairing;

    // Starting ID for uniquely numbering all RoutePlanRequest messages
    int64_t m_routePlanRequestId{1};
    
    // Store RoutePlanResponse messages received so far
    // Map < RoutePlanResponse.ResponseID, RoutePlanResponse >
    std::unordered_map<int64_t, std::shared_ptr<uxas::messages::route::RoutePlanResponse> > m_routePlanResponses;

    // When a RouteRequest is processed, this map stores the IDs of all RoutePlanResponse messages 
    // that must be received before a corresponding RouteResponse can be created
    // Map < RouteRequest.RequestID, Set <RoutePlanResponse.ResponseID> >
    std::unordered_map<int64_t, std::unordered_set<int64_t> > m_routeRequestIdToPendingRoutePlanResponseIds;
};

}; //namespace service
}; //namespace uxas

#endif /* UXAS_SERVICE_ROUTE_AGGREGATOR_NEW_SERVICE_H */
