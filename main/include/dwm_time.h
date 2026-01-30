#ifndef DW1000TIME_H
#define DW1000TIME_H

#include <stdint.h>
#include <inttypes.h>

class DW1000Time {
public:
	// Time resolution in micro-seconds of time based registers/values.
	// Each bit in a timestamp counts for a period of approx. 15.65ps
	static constexpr float TIME_RES     = 0.000015650040064103f;
	static constexpr float TIME_RES_INV = 63897.6f;
	
	// Speed of radio waves [m/s] * timestamp resolution [~15.65ps] of DW1000
	static constexpr float DISTANCE_OF_RADIO     = 0.0046917639786159f;
	static constexpr float DISTANCE_OF_RADIO_INV = 213.139451293f;
	
	// timestamp byte length - 40 bit -> 5 byte
	static constexpr uint8_t LENGTH_TIMESTAMP = 5;
	
	// timer/counter overflow (40 bits) -> 4overflow approx. every 17.2 seconds
	static constexpr int64_t TIME_OVERFLOW = 0x10000000000; //1099511627776LL
	static constexpr int64_t TIME_MAX      = 0xffffffffff;
	
	// time factors (relative to [us]) for setting delayed transceive
	// TODO use non float
	static constexpr float SECONDS      = 1e6;
	static constexpr float MILLISECONDS = 1e3;
	static constexpr float MICROSECONDS = 1;
	static constexpr float NANOSECONDS  = 1e-3;
	
	// constructor
	DW1000Time();
	DW1000Time(int64_t time);
	DW1000Time(uint8_t data[]);
	DW1000Time(const DW1000Time& copy);
	DW1000Time(float timeUs);
	DW1000Time(int32_t value, float factorUs);
	~DW1000Time();
	
	// setter
	// dw1000 timestamp, increase of +1 approx approx. 15.65ps real time
	void setTimestamp(int64_t value);
	void setTimestamp(uint8_t data[]);
	void setTimestamp(const DW1000Time& copy);
	
	// real time in us
	void setTime(float timeUs);
	void setTime(int32_t value, float factorUs);
	
	// getter
	int64_t getTimestamp() const;
	void    getTimestamp(uint8_t data[]) const;
	
	float getAsFloat() const;
	// getter, convert the timestamp to usual units
	float getAsMicroSeconds() const;
	//void getAsuint8_ts(uint8_t data[]) const; // TODO check why it is here, is it old version of getTimestamp(uint8_t) ?
	float getAsMeters() const;
	
	DW1000Time& wrap();
	
	// self test
	bool isValidTimestamp();
	
	// assign
	DW1000Time& operator=(const DW1000Time& assign);
	// add
	DW1000Time& operator+=(const DW1000Time& add);
	DW1000Time operator+(const DW1000Time& add) const;
	// subtract
	DW1000Time& operator-=(const DW1000Time& sub);
	DW1000Time operator-(const DW1000Time& sub) const;
	// multiply
	// multiply with float cause lost in accuracy, because float calculates only with 23bit matise
	DW1000Time& operator*=(float factor);
	DW1000Time operator*(float factor) const;
	// no accuracy lost
	DW1000Time& operator*=(const DW1000Time& factor);
	DW1000Time operator*(const DW1000Time& factor) const;
	// divide
	// divide with float cause lost in accuracy, because float calculates only with 23bit matise
	DW1000Time& operator/=(float factor);
	DW1000Time operator/(float factor) const;
	// no accuracy lost
	DW1000Time& operator/=(const DW1000Time& factor);
	DW1000Time operator/(const DW1000Time& factor) const;
	// compare
	bool operator==(const DW1000Time& cmp) const;
	bool operator!=(const DW1000Time& cmp) const;


	
private:
	// timestamp size from dw1000 is 40bit, maximum number 1099511627775
	// signed because you can calculate with DW1000Time; negative values are possible errors
	int64_t _timestamp = 0;
};

#endif // DW1000Time_H
