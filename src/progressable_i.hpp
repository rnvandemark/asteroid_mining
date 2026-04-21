#pragma once

namespace am {

class ProgressableI
{
public:
    virtual void progress_over(const double dt) = 0;
};

}
