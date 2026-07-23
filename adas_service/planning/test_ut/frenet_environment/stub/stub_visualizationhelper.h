#ifndef STUB_VISUALIZATIONHELPER_H
#define STUB_VISUALIZATIONHELPER_H

class FrenetEnvironment; 

class VisualizationHelper
{
public:
    VisualizationHelper(FrenetEnvironment* context) {};
    void visualize() {};
    void resetCanvas() {};
    void setContext(FrenetEnvironment* new_context) {};
};

#endif