#pragma once
#include <thread>
class thread_guard
{
private:
	std::thread& t;
public:
	explicit thread_guard(std::thread& _t) :t(_t) {}
	~thread_guard()
	{
		if (t.joinable())
			t.join();
	}
private:
	thread_guard(thread_guard const&);
	//thread_guard& operator=(thread_guard const&);

};