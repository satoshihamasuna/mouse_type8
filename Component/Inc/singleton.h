/*
 * singleton.h
 *
 *  Created on: Jul 4, 2025
 *      Author: sato1
 */

#ifndef INC_SINGLETON_H_
#define INC_SINGLETON_H_


template <class T>
class Singleton {
public:
	static T& getInstance(void) {
		static T _instance;
		return _instance;
	}

protected:
	Singleton() = default;
	~Singleton() = default;

private:
	Singleton(const Singleton&) = delete;
	Singleton& operator=(const Singleton&) = delete;
	Singleton(Singleton&&) = delete;
	Singleton& operator=(Singleton&&) = delete;
};


#endif /* INC_SINGLETON_H_ */
