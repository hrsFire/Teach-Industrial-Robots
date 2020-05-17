#include "simple_gestures/gestures/gestures_engine.hpp"

using namespace gestures;

GesturesEngine::GesturesEngine(GesturesBase* gesturesImpl) : gesturesImpl(gesturesImpl) {
}

GesturesEngine::~GesturesEngine() {
    delete gesturesImpl;
}

void GesturesEngine::AddGesture(Gesture gesture, std::string group, uint groupPriority, uint priorityInGroup, std::vector<std::string> affectedItems, bool preventConflicts) {
    bool wasAdded = false;
    std::list<GestureGroup>::iterator gestureGroupItr;
    GestureItem gestureItem = GestureItem(gesture, affectedItems, preventConflicts);

    // Search for a group which is already available
    for (gestureGroupItr = groups.begin(); gestureGroupItr != groups.end() && gestureGroupItr->name != group; gestureGroupItr++);

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
    } else {
        // The group isn't available yet
        std::list<GestureItem> gestures;
        gestures.push_back(gestureItem);
        GestureGroup gestureGroup = GestureGroup(group, gestures);

        uint i = 0;
        for (gestureGroupItr = groups.begin(); i != groupPriority && gestureGroupItr != groups.end(); gestureGroupItr++, i++);

        if (gestureGroupItr != groups.end()) {
            groups.insert(gestureGroupItr, gestureGroup);
        } else {
            groups.push_back(gestureGroup);
        }
    }
}

void GesturesEngine::Start() {
    isRunning = true;
    std::unordered_set<std::string> affectedItems;
    bool isAffected;
    bool isCleanupActive = false;

    while (isRunning) {
        gesturesImpl->NextCycle();

        if (gesturesImpl->IsNewDataAvailable()) {
            for (GestureGroup& group : groups) {
                isCleanupActive = false;

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
                            std::chrono::system_clock::time_point currentTime = std::chrono::system_clock::now();

                            if (!gestureItem.isActive) {
                                gestureItem.startTime = currentTime;
                                gestureItem.isActive = true;
                            }

                            std::chrono::seconds gestureDuration = std::chrono::duration_cast<std::chrono::seconds>(currentTime - gestureItem.startTime);

#ifndef NDEBUG
                            std::cout << "Gesture Duration: " << gestureDuration.count() << " s" << std::endl;
#endif

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

            affectedItems.clear();
            gesturesImpl->MarkDataAsUsed();
        }
    }
}

void GesturesEngine::Stop() {
    isRunning = false;
}