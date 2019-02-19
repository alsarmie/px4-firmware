#pragma once

struct Error {
private:
	int _num;
	const char *_msg;
	operator bool() { return false; }
public:
	Error(int num_, const char *msg_) : _num{num_}, _msg{msg_} {}
	Error(const char *msg_) : _num{msg_ == nullptr ? 0 : -1}, _msg{msg_} {}
	Error() : _num{0}, _msg{"ok"} {}
	Error(bool ok) : _num{ok ? 0 : -1}, _msg{ok ? "ok" : "error"} {}
	operator const char *() { return _msg; }
	bool ok() { return _num == 0; }
	int num() { return _num; }
	const char *msg() { return _msg; }
};
