
#pragma once

namespace zapata {

	template <class T> class  StdVector
	{
	public:

		typedef T * iterator;

		StdVector();
		StdVector(unsigned int size);
		StdVector(unsigned int size, const T & initial);
		StdVector(const StdVector<T> & v);
		~StdVector();

		unsigned int capacity() const;
		unsigned int size() const;
		bool empty() const;
		iterator begin();
		iterator end();
		T & front();
		T & back();
		void push_back(const T & value);
		void pop_back();

		void reserve(unsigned int capacity);
		void resize(unsigned int size);

		T & operator[](unsigned int index);
		StdVector<T> & operator=(const StdVector<T> &);
		void clear();
		private:
		unsigned int my_size;
		unsigned int my_capacity;
		T * buffer;
	};


	template<class T>
	StdVector<T>::StdVector()
	{
	my_capacity = 0;
	my_size = 0;
	buffer = 0;
	}

	template<class T>
	StdVector<T>::StdVector(const StdVector<T> & v)
	{
	my_size = v.my_size;
	my_capacity = v.my_capacity;
	buffer = new T[my_size];
	for (unsigned int i = 0; i < my_size; i++)
		buffer[i] = v.buffer[i];
	}

	template<class T>
	StdVector<T>::StdVector(unsigned int size)
	{
	my_capacity = size;
	my_size = size;
	buffer = new T[size];
	}

	template<class T>
	StdVector<T>::StdVector(unsigned int size, const T & initial)
	{
	my_size = size;
	my_capacity = size;
	buffer = new T [size];
	for (unsigned int i = 0; i < size; i++)
		buffer[i] = initial;
	//T();
	}

	template<class T>
	StdVector<T> & StdVector<T>::operator = (const StdVector<T> & v)
	{
	delete[ ] buffer;
	my_size = v.my_size;
	my_capacity = v.my_capacity;
	buffer = new T [my_size];
	for (unsigned int i = 0; i < my_size; i++)
		buffer[i] = v.buffer[i];
	return *this;
	}

	template<class T>
	typename StdVector<T>::iterator StdVector<T>::begin()
	{
	return buffer;
	}

	template<class T>
	typename StdVector<T>::iterator StdVector<T>::end()
	{
	return buffer + size();
	}

	template<class T>
	T& StdVector<T>::front()
	{
	return buffer[0];
	}

	template<class T>
	T& StdVector<T>::back()
	{
	return buffer[my_size - 1];
	}

	template<class T>
	void StdVector<T>::push_back(const T & v)
	{
	if (my_size >= my_capacity)
		reserve(my_capacity +5);
	buffer [my_size++] = v;
	}

	template<class T>
	void StdVector<T>::pop_back()
	{
	my_size--;
	}

	template<class T>
	void StdVector<T>::reserve(unsigned int capacity)
	{
	if(buffer == 0)
	{
		my_size = 0;
		my_capacity = 0;
	}
	T * Newbuffer = new T [capacity];
	//assert(Newbuffer);
	unsigned int l_Size = capacity < my_size ? capacity : my_size;
	//copy (buffer, buffer + l_Size, Newbuffer);

	for (unsigned int i = 0; i < l_Size; i++)
		Newbuffer[i] = buffer[i];

	my_capacity = capacity;
	delete[] buffer;
	buffer = Newbuffer;
	}

	template<class T>
	unsigned int StdVector<T>::size()const//
	{
	return my_size;
	}

	template<class T>
	void StdVector<T>::resize(unsigned int size)
	{
	reserve(size);
	my_size = size;
	}

	template<class T>
	T& StdVector<T>::operator[](unsigned int index)
	{
	return buffer[index];
	}

	template<class T>
	unsigned int StdVector<T>::capacity()const
	{
	return my_capacity;
	}

	template<class T>
	StdVector<T>::~StdVector()
	{
	delete[ ] buffer;
	}
	template <class T>
	void StdVector<T>::clear()
	{
	my_capacity = 0;
	my_size = 0;
	buffer = 0;
	};


	template <class T> int partition(StdVector<T> &values, int left, int right) {
		int pivotIndex = left + (right - left) / 2;
		T pivotValue = values[pivotIndex];
		int i = left, j = right;
		T temp;
		while(i <= j) {
			while(values[i] < pivotValue) {
				i++;
			}
			while(values[j] > pivotValue) {
				j--;
			}
			if(i <= j) {
				temp = values[i];
				values[i] = values[j];
				values[j] = temp;
				i++;
				j--;
			}
		}
		return i;
	}

	template <class T> void quicksort(StdVector<T> &values, int left, int right) {
		if(left < right) {
			int pivotIndex = partition(values, left, right);
			quicksort(values, left, pivotIndex - 1);
			quicksort(values, pivotIndex, right);
		}
	}

	int getStdVectorVersion();
}

