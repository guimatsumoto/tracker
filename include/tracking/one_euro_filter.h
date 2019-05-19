/* -*- coding: utf-8 -*-
 * Inspired by nicolas.roussel@inria.fr
 */

#include <iostream>
#include <stdexcept>
#include <cmath>
#include <ctime>
#include "sl_core/ai/ai_release.hpp"

namespace zed_tracking {

    class  LowPassFilter {
    public:

        LowPassFilter(double alpha, double initval = 0.0) {
            reset(initval);
            setAlpha(alpha);
        }

        void reset(double initval = 0.0) {
            y = s = initval;
            initialized = false;
        }

        double filter(double value) {
            double result;
            if (initialized)
                result = a * value + (1.0 - a) * s;
            else {
                result = value;
                initialized = true;
            }
            y = value;
            s = result;
            return result;
        }

        double filterWithAlpha(double value, double alpha) {
            setAlpha(alpha);
            return filter(value);
        }

        bool hasLastRawValue(void) {
            return initialized;
        }

        double lastRawValue(void) {
            return y;
        }

        double lastFilterValue(void) {
            return s;
        }

    private:

        double y, a, s;
        bool initialized;

        void setAlpha(double alpha) {
            if (alpha <= 0.0 || alpha > 1.0)
                throw std::range_error("alpha should be in (0.0., 1.0]");
            a = alpha;
        }

    };



#define OneEuroUndefinedTime -1.0

    class  OneEuroFilter {
    public:

        OneEuroFilter(double freq,
                double mincutoff = 0.3, double beta_ = 20, double dcutoff = 1.0) {
            setFrequency(freq);
            setMinCutoff(mincutoff);
            setBeta(beta_);
            setDerivateCutoff(dcutoff);
            x = new LowPassFilter(alpha(mincutoff));
            dx = new LowPassFilter(alpha(dcutoff));
            lasttime = OneEuroUndefinedTime;
        }

        /*
         * Timestamp in seconds
         */
        double update(double value, double timestamp, bool reset = false) {
            if (reset) {
                x->reset(value);
                dx->reset(value);
                lasttime = timestamp;
            }

            double return_value = x->lastFilterValue();

            if (timestamp > lasttime) {
                if (lasttime != OneEuroUndefinedTime) {
                    freq = 1.0 / (timestamp - lasttime);
                    lasttime = timestamp;
                } else
                    lasttime = timestamp;

                // estimate the current variation per second 
                double dvalue = x->hasLastRawValue() ? (value - x->lastRawValue()) * freq : 0.0; // FIXME: 0.0 or value?
                double edvalue = dx->filterWithAlpha(dvalue, alpha(dcutoff));

                // use it to update the cutoff frequency
                double cutoff = mincutoff + beta_ * fabs(edvalue);

                // filter the given value
                return_value = x->filterWithAlpha(value, alpha(cutoff));
            }


            return return_value;
        }

        ~OneEuroFilter(void) {
            delete x;
            delete dx;
        }

    private:
        double freq;
        double mincutoff;
        double beta_;
        double dcutoff;
        LowPassFilter *x;
        LowPassFilter *dx;
        double lasttime;

        double alpha(double cutoff) {
            double te = 1.0 / freq;
            double tau = 1.0 / (2 * 3.14159265359 * cutoff);
            return 1.0 / (1.0 + tau / te);
        }

        void setFrequency(double f) {
            if (f <= 0) throw std::range_error("freq should be >0");
            freq = f;
        }

        void setMinCutoff(double mc) {
            if (mc <= 0) throw std::range_error("mincutoff should be >0");
            mincutoff = mc;
        }

        void setBeta(double b) {
            beta_ = b;
        }

        void setDerivateCutoff(double dc) {
            if (dc <= 0) throw std::range_error("dcutoff should be >0");
            dcutoff = dc;
        }
    };

    class  OneEuroFilter3D {
    public:

        OneEuroFilter3D(double freq,
                double mincutoff = 1.0, double beta_ = 0.0, double dcutoff = 1.0) {
            setFrequency(freq);
            setMinCutoff(mincutoff);
            setBeta(beta_);
            setDerivateCutoff(dcutoff);
            x = new LowPassFilter(alpha(mincutoff));
            dx = new LowPassFilter(alpha(dcutoff));
            y = new LowPassFilter(alpha(mincutoff));
            dy = new LowPassFilter(alpha(dcutoff));
            z = new LowPassFilter(alpha(mincutoff));
            dz = new LowPassFilter(alpha(dcutoff));
            lasttime = OneEuroUndefinedTime;
        }

        /*
         * Timestamp in seconds
         */
        void update(double value_x, double value_y, double value_z, double timestamp, bool reset = false) {
            if (reset) {
                x->reset(value_x);
                dx->reset(value_x);
                y->reset(value_y);
                dy->reset(value_y);
                z->reset(value_z);
                dz->reset(value_z);
                lasttime = timestamp;
            }

            double return_value_x = x->lastFilterValue();
            double return_value_y = y->lastFilterValue();
            double return_value_z = z->lastFilterValue();

            if (timestamp > lasttime) {
                if (lasttime != OneEuroUndefinedTime) {
                    freq = 1.0 / (timestamp - lasttime);
                    lasttime = timestamp;
                } else
                    lasttime = timestamp;

                // estimate the current x variation per second 
                double dvalue_x = x->hasLastRawValue() ? (value_x - x->lastRawValue()) * freq : 0.0; // FIXME: 0.0 or value?
                double edvalue_x = dx->filterWithAlpha(dvalue_x, alpha(dcutoff));

                // estimate the current y variation per second 
                double dvalue_y = y->hasLastRawValue() ? (value_y - y->lastRawValue()) * freq : 0.0; // FIXME: 0.0 or value?
                double edvalue_y = dy->filterWithAlpha(dvalue_y, alpha(dcutoff));

                // estimate the current z variation per second 
                double dvalue_z = z->hasLastRawValue() ? (value_z - z->lastRawValue()) * freq : 0.0; // FIXME: 0.0 or value?
                double edvalue_z = dz->filterWithAlpha(dvalue_z, alpha(dcutoff));

                // use it to update the cutoff frequency
                double cutoff_x = mincutoff + beta_ * fabs(edvalue_x);
                double cutoff_y = mincutoff + beta_ * fabs(edvalue_y);
                double cutoff_z = mincutoff + beta_ * fabs(edvalue_z);

                // filter the given value
                return_value_x = x->filterWithAlpha(value_x, alpha(cutoff_x));
                return_value_y = y->filterWithAlpha(value_y, alpha(cutoff_y));
                return_value_z = z->filterWithAlpha(value_z, alpha(cutoff_z));
            }

        }

        void update(double timestamp) {
            // Verify if the blind update was done on a new frame
            if (timestamp > lasttime) {
                double new_x = x->lastFilterValue();
                double new_y = y->lastFilterValue();
                double new_z = z->lastFilterValue();
                double last_vx = dx->lastFilterValue();
                double last_vy = dy->lastFilterValue();
                double last_vz = dz->lastFilterValue();

                // estimate new x position
                new_x = new_x + last_vx * (timestamp - lasttime);

                // estimate new y position
                new_y = new_y + last_vy * (timestamp - lasttime);

                // estimate new z position
                new_z = new_z + last_vz * (timestamp - lasttime);

                // update on predicted new state
                update(new_x, new_y, new_z, timestamp, false);
            }

        }

        void getState(double &value_x, double &value_y, double &value_z) {
            value_x = x->lastFilterValue();
            value_y = y->lastFilterValue();
            value_z = z->lastFilterValue();
        }

        void getFullState(double &value_x, double &value_y, double &value_z,
                double &vx, double &vy, double &vz) {
            value_x = x->lastFilterValue();
            value_y = y->lastFilterValue();
            value_z = z->lastFilterValue();
            vx = dx->lastFilterValue();
            vy = dy->lastFilterValue();
            vz = dz->lastFilterValue();
        }

        ~OneEuroFilter3D(void) {
            delete x;
            delete dx;
            delete y;
            delete dy;
            delete z;
            delete dz;
        }

    private:
        double freq;
        double mincutoff;
        double beta_;
        double dcutoff;
        LowPassFilter *x;
        LowPassFilter *dx;
        LowPassFilter *y;
        LowPassFilter *dy;
        LowPassFilter *z;
        LowPassFilter *dz;
        double lasttime;

        double alpha(double cutoff) {
            double te = 1.0 / freq;
            double tau = 1.0 / (2 * 3.14159265359 * cutoff);
            return 1.0 / (1.0 + tau / te);
        }

        void setFrequency(double f) {
            if (f <= 0) throw std::range_error("freq should be >0");
            freq = f;
        }

        void setMinCutoff(double mc) {
            if (mc <= 0) throw std::range_error("mincutoff should be >0");
            mincutoff = mc;
        }

        void setBeta(double b) {
            beta_ = b;
        }

        void setDerivateCutoff(double dc) {
            if (dc <= 0) throw std::range_error("dcutoff should be >0");
            dcutoff = dc;
        }
    };

    class  FixedFreqOneEuroFilter {
    public:

        FixedFreqOneEuroFilter(double freq,
                double mincutoff = 1.0, double beta_ = 0.0, double dcutoff = 1.0) {
            setFrequency(freq);
            setMinCutoff(mincutoff);
            setBeta(beta_);
            setDerivateCutoff(dcutoff);
            x = new LowPassFilter(alpha(mincutoff));
            dx = new LowPassFilter(alpha(dcutoff));
            is_init = false;
        }

        /*
         * Timestamp in seconds
         */
        double update(double value, bool reset = false) {

            if (reset || !is_init) {
                x->reset(value);
                dx->reset(value);

                is_init = false;
            }

            double return_value = x->lastFilterValue();

            if (is_init) {
                // estimate the current variation per second 
                double dvalue = x->hasLastRawValue() ? (value - x->lastRawValue()) * freq : 0.0; // FIXME: 0.0 or value?
                double edvalue = dx->filterWithAlpha(dvalue, alpha(dcutoff));

                // use it to update the cutoff frequency
                double cutoff = mincutoff + beta_ * fabs(edvalue);

                // filter the given value
                return_value = x->filterWithAlpha(value, alpha(cutoff));
            } else
                is_init = true;


            return return_value;
        }

        ~FixedFreqOneEuroFilter(void) {
            delete x;
            delete dx;
        }

    private:
        double freq;
        double mincutoff;
        double beta_;
        double dcutoff;
        LowPassFilter *x;
        LowPassFilter *dx;
        bool is_init;

        double alpha(double cutoff) {
            double te = 1.0 / freq;
            double tau = 1.0 / (2 * 3.14159265359 * cutoff);
            return 1.0 / (1.0 + tau / te);
        }

        void setFrequency(double f) {
            if (f <= 0) throw std::range_error("freq should be >0");
            freq = f;
        }

        void setMinCutoff(double mc) {
            if (mc <= 0) throw std::range_error("mincutoff should be >0");
            mincutoff = mc;
        }

        void setBeta(double b) {
            beta_ = b;
        }

        void setDerivateCutoff(double dc) {
            if (dc <= 0) throw std::range_error("dcutoff should be >0");
            dcutoff = dc;
        }
    };

    // -----------------------------------------------------------------
    /*
    int
    main(int argc, char **argv) {
        randSeed();

        double duration = 10.0; // seconds

        double frequency = 120; // Hz
        double mincutoff = 1.0; // FIXME
        double beta = 1.0;      // FIXME
        double dcutoff = 1.0;   // this one should be ok

        std::cout << "#SRC OneEuroFilter.cc" << std::endl
            << "#CFG {'beta': " << beta << ", 'freq': " << frequency << ", 'dcutoff': " << dcutoff << ", 'mincutoff': " << mincutoff << "}" << std::endl
            << "#LOG timestamp, signal, noisy, filtered" << std::endl;

        OneEuroFilter f(frequency, mincutoff, beta, dcutoff);
        for (double timestamp = 0.0; timestamp<duration; timestamp += 1.0 / frequency) {
            double signal = sin(timestamp);
            double noisy = signal + (unifRand() - 0.5) / 5.0;
            double filtered = f.filter(noisy, timestamp);
            std::cout << timestamp << ", "
                << signal << ", "
                << noisy << ", "
                << filtered
                << std::endl;
        }

        return 0;
    }*/
} /*namespace zed_tracking*/
