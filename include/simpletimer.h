#pragma once

#include <chrono>

class SimpleTimer
{
public:
	SimpleTimer()
	{
		startTimePoint = std::chrono::system_clock::now();
		endTimePoint = startTimePoint;
	}

	void start()
	{
		startTimePoint = std::chrono::system_clock::now();
	}

	void stop()
	{
		endTimePoint = std::chrono::system_clock::now();
	}

	int getMilliseconds()
	{
		auto ret = std::chrono::duration_cast<std::chrono::milliseconds>(endTimePoint - startTimePoint);
		return (int)(ret.count());
	}

	void printLapTimeAndRestart(const char *text)
	{
		endTimePoint = std::chrono::system_clock::now();
		auto ret = std::chrono::duration_cast<std::chrono::milliseconds>(endTimePoint - startTimePoint);
		printf("\n%s: %d [ms]", text, (int)ret.count());
		start(); 
	}

private:
	std::chrono::time_point<std::chrono::system_clock> startTimePoint, endTimePoint;

};