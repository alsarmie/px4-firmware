#pragma once

struct Error {
	int num;
	const char *msg;
	Error(int num_, const char *msg_) : num{num_}, msg{msg_} {}
	Error(const char *msg_) : num{-1}, msg{msg_} {}
	Error() : num{0}, msg{"success"} {}
	Error(bool ok) : num{ok ? 0 : -1}, msg{"success"} {}
	operator const char *() { return msg; }
	operator bool() { return num != 0; }
};
