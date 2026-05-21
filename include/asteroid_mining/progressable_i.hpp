#pragma once

namespace asteroid_mining {

class ProgressableI
{
public:
    virtual void progress_over(const double dt) = 0;
};

}
