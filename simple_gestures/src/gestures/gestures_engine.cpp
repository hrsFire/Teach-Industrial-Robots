#define GESTURES_GESTURES_ENGINE_PRIVATE

#include "simple_gestures/gestures/gestures_engine.hpp"
#include "gesture_group_item.hpp"
#include <chrono>
#include <thread>

#ifdef MEASUREMENT
#include <iostream>
#include <fstream>
#include <sstream>
#endif //MEASUREMENT

using namespace gestures;

GesturesEngine::GesturesEngine(GesturesBase* gesturesImpl) : gesturesImpl(gesturesImpl) {
    this->groups = reinterpret_cast<std::list<void>*>(new std::list<GestureGroupItem>);
}

GesturesEngine::~GesturesEngine() {
    delete gesturesImpl;
    delete reinterpret_cast<std::list<GestureGroupItem>*>(groups);
}

GestureGroup GesturesEngine::AddGestureGroup(std::string name, uint priority, std::list<GestureGroup> excludedGroups) {
    std::list<GestureGroupItem>::iterator gestureGroupItr;
    std::list<GestureGroupItem>& groups = *reinterpret_cast<std::list<GestureGroupItem>*>(this->groups);

    // Search for a group which is already available
    for (gestureGroupItr = groups.begin(); gestureGroupItr != groups.end() && gestureGroupItr->name != name; gestureGroupItr++);

    if (gestureGroupItr != groups.end()) {
        // The group is already available. Therefore don't insert the group.
        return GestureGroup(gestureGroupItr->name);
    } else {
        // The group isn't available yet. Therefore create the group.
        GestureGroupItem gestureGroupItem = GestureGroupItem(name, {}, excludedGroups);
        uint i = 0;
        for (gestureGroupItr = groups.begin(); i != priority && gestureGroupItr != groups.end(); gestureGroupItr++, i++);

        if (gestureGroupItr != groups.end()) {
            groups.insert(gestureGroupItr, gestureGroupItem);
        } else {
            groups.push_back(gestureGroupItem);
        }

        return GestureGroup(name);
    }
}

bool GesturesEngine::AddGesture(Gesture gesture, GestureGroup group, uint priorityInGroup, std::vector<std::string> affectedItems, bool preventConflicts) {
    bool wasAdded = false;
    std::list<GestureGroupItem>::iterator gestureGroupItr;
    GestureItem gestureItem = GestureItem(gesture, affectedItems, preventConflicts);
    std::list<GestureGroupItem>& groups = *reinterpret_cast<std::list<GestureGroupItem>*>(this->groups);

    // Search for a group which is already available
    for (gestureGroupItr = groups.begin(); gestureGroupItr != groups.end() && gestureGroupItr->name != (std::string) group; gestureGroupItr++);

    if (gestureGroupItr != groups.end()) {
        // The group is already available. Therefore insert the gesture at the correct position.
        std::list<GestureItem>::iterator gestureItemItr;
        uint i = 0;
        for (gestureItemItr = gestureGroupItr->gestures.begin(); i < priorityInGroup && gestureItemItr != gestureGroupItr->gestures.end(); gestureItemItr++);

        if (gestureItemItr != gestureGroupItr->gestures.end()) {
            gestureGroupItr->gestures.insert(gestureItemItr, gestureItem);
        } else {
            gestureGroupItr->gestures.push_back(gestureItem);
        }

        return true;
    } else {
        // The group isn't available yet
        return false;
    }
}

void GesturesEngine::Start() {
    isRunning = true;
    std::unordered_set<std::string> affectedItems;
    std::vector<GestureGroup> executedGroups;
    bool isAffected;
    bool isCleanupActive = false;
    std::list<GestureGroupItem>& groups = *reinterpret_cast<std::list<GestureGroupItem>*>(this->groups);

#ifdef MEASUREMENT
    std::ofstream measurementFile("gestures_recognition_measurement.csv", std::ios::out | std::ios::trunc);
    measurementFile << "Time" << ", " << "Duration (ms)" << std::endl;
    std::chrono::system_clock::time_point lastTime = std::chrono::system_clock::now();
    std::chrono::milliseconds timeDuration;
    std::chrono::system_clock::time_point currentTime = lastTime;
    std::time_t time;
    std::string timeString;
#endif //MEASUREMENT

    while (isRunning) {
        gesturesImpl->NextCycle();

        if (gesturesImpl->IsNewDataAvailable()) {
#ifdef MEASUREMENT
            if (currentTime == lastTime) {
                lastTime = std::chrono::system_clock::now();
            } else {
                currentTime = std::chrono::system_clock::now();
                timeDuration = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastTime);
                time = std::chrono::system_clock::to_time_t(currentTime);
                timeString = std::ctime(&time);
                measurementFile << timeString.substr(0, timeString.length() -1) << ", " << timeDuration.count() << std::endl;
                measurementFile.flush();
                lastTime = currentTime;
            }
#endif //MEASUREMENT

            for (GestureGroupItem& group : groups) {
                isCleanupActive = false;

                for (const GestureGroup& executedGroup : executedGroups) {
                    for (const GestureGroup& gestureGroup : group.excludedGroups) {
                        if (executedGroup == gestureGroup) {
                            isCleanupActive = true;
                            break;
                        }
                    }

                    if (isCleanupActive) {
                        break;
                    }
                }

                for (GestureItem& gestureItem : group.gestures) {
                    if (!isCleanupActive) {
                        isAffected = false;

                        if (gestureItem.preventConflicts) {
                            for (const std::string& affectedItem : gestureItem.affectedItems) {
                                if (affectedItems.find(affectedItem) != affectedItems.end()) {
                                    isAffected = true;
                                    break;
                                }
                            }

                            if (isAffected) {
                                gestureItem.isActive = false;
                                continue;
                            }
                        }

                        if (gestureItem.gesture.isGesture(*gesturesImpl)) {
                            executedGroups.push_back(GestureGroup(group.name));
                            std::chrono::system_clock::time_point currentTime = std::chrono::system_clock::now();

                            if (!gestureItem.isActive) {
                                gestureItem.startTime = currentTime;
                                gestureItem.isActive = true;
                            }

                            std::chrono::seconds gestureDuration = std::chrono::duration_cast<std::chrono::seconds>(currentTime - gestureItem.startTime);

#ifndef NDEBUG
                            std::cout << "Gesture Duration: " << gestureDuration.count() << " s" << std::endl;
#endif //NDEBUG

                            gestureItem.gesture.doAction(gestureDuration);

                            for (const std::string& affectedItem : gestureItem.affectedItems) {
                                affectedItems.insert(affectedItem);
                            }

                            // Only one element in a group can be active
                            isCleanupActive = true;
                        } else {
                            // If the gesture wasn't recognized once the gesture is set to inactive
                            gestureItem.isActive = false;
                        }
                    } else {
                        gestureItem.isActive = false;
                    }
                }
            }

            executedGroups.clear();
            affectedItems.clear();
            gesturesImpl->MarkDataAsUsed();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void GesturesEngine::Stop() {
    isRunning = false;
}