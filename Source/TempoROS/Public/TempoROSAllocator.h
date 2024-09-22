// Copyright Tempo Simulation, LLC. All Rights Reserved

#pragma once

#include <memory_resource>

// template <class T>
// struct TempoROSAllocator {
// 	using value_type = T;
// 	TempoROSAllocator() noexcept = default;
// 	template <class U> TempoROSAllocator (const TempoROSAllocator<U>&) noexcept {}
// 	T* allocate (std::size_t n)
// 	{
// 		return FMemory::Malloc(sizeof(T) * n);
// 	}
// 	void deallocate (T* p, std::size_t n)
// 	{
// 		FMemory::Free(p);
// 	}
// };
//
// template <class T, class U>
// constexpr bool operator== (const TempoROSAllocator<T>&, const TempoROSAllocator<U>&) noexcept { return true; }
//
// template <class T, class U>
// constexpr bool operator!= (const TempoROSAllocator<T>&, const TempoROSAllocator<U>&) noexcept { return false; }

#include <rcutils/allocator.h>

#include "HAL/UnrealMemory.h"

// template <typename T>
// struct UnrealEngineAllocator {
// 	using value_type = T;
//     
	// T* allocate(std::size_t n) {
	// 	return static_cast<T*>(FMemory::Malloc(n * sizeof(T)));
	// }
 //    
	// void deallocate(T* p, std::size_t n) {
	// 	FMemory::Free(p);
	// }
//     
// 	// Add other necessary members to conform to C++ Allocator requirements
// 	// such as construct, destroy, max_size, etc.
// };

// template <class T>
// struct UnrealEngineAllocator {
// 	using value_type = T;
// 	using size_type = std::size_t;
// 	using difference_type = std::ptrdiff_t;
// 	using propagate_on_container_move_assignment = std::true_type;
// 	using is_always_equal = std::true_type;
//
// 	UnrealEngineAllocator() noexcept = default;
// 	template <typename U> UnrealEngineAllocator(const UnrealEngineAllocator<U>&) noexcept {}
// 	
	// T* allocate(std::size_t n) {
	// 	return static_cast<T*>(FMemory::Malloc(n * sizeof(T)));
	// }
	// void deallocate(T* p, std::size_t n) {
	// 	FMemory::Free(p);
	// }
// };

// template <class T, class U>
// constexpr bool operator== (const UnrealEngineAllocator<T>&, const UnrealEngineAllocator<U>&) noexcept { return true; }
//
// template <class T, class U>
// constexpr bool operator!= (const UnrealEngineAllocator<T>&, const UnrealEngineAllocator<U>&) noexcept { return false; }

// template<typename T>
// struct pointer_traits {
// 	using reference = T &;
// 	using const_reference = const T &;
// };
//
// // Avoid declaring a reference to void with an empty specialization
// template<>
// struct pointer_traits<void> {
// };
//
// template<typename T = void>
// struct UnrealEngineAllocator : public pointer_traits<T> {
// public:
// 	using value_type = T;
// 	using size_type = std::size_t;
// 	using pointer = T *;
// 	using const_pointer = const T *;
// 	using difference_type = typename std::pointer_traits<pointer>::difference_type;
//
// 	UnrealEngineAllocator() noexcept = default;
//
// 	~UnrealEngineAllocator() noexcept = default;
//
// 	template<typename U>
// 	UnrealEngineAllocator(const UnrealEngineAllocator<U> &) noexcept {};
// 	
// 	T* allocate(std::size_t n) {
// 		return static_cast<T*>(FMemory::Malloc(n * sizeof(T)));
// 	}
// 	void deallocate(T* p, std::size_t n) {
// 		FMemory::Free(p);
// 	}
//
// 	template<typename U>
// 	struct rebind {
// 		typedef UnrealEngineAllocator<U> other;
// 	};
// };
//
// template <>
// struct UnrealEngineAllocator<void> {
// 	using value_type = void;
// 	using pointer = void*;
// 	using const_pointer = const void*;
// 	template <typename U> struct rebind { using other = UnrealEngineAllocator<U>; };
//
// 	UnrealEngineAllocator() = default;
// 	template <typename U> UnrealEngineAllocator(const UnrealEngineAllocator<U>&) {}
// };
//
// template<typename T, typename U>
// constexpr bool operator==(const UnrealEngineAllocator<T> &,
//   const UnrealEngineAllocator<U> &) noexcept { return true; }
//
// template<typename T, typename U>
// constexpr bool operator!=(const UnrealEngineAllocator<T> &,
//   const UnrealEngineAllocator<U> &) noexcept { return false; }
//
 inline void* UnrealAlloc(size_t size, void* state)
 {
 	return FMemory::Malloc(size);
 }

 inline void UnrealDealloc(void* pointer, void* state)
 {
 	FMemory::Free(pointer);
 }

 inline void* UnrealRealloc(void* pointer, size_t size, void* state)
 {
 	return FMemory::Realloc(pointer, size);
 }

 inline void* UnrealZeroAlloc(size_t number_of_elements, size_t size_of_element, void* state)
 {
 	size_t size = number_of_elements * size_of_element;
 	void* ptr = FMemory::Malloc(size);
 	FMemory::Memzero(ptr, size);
 	return ptr;
 }

 inline rcl_allocator_t GetUnrealAllocator()
 {
 	rcutils_allocator_t UnrealAllocator;
 	UnrealAllocator.allocate = UnrealAlloc;
 	UnrealAllocator.deallocate = UnrealDealloc;
 	UnrealAllocator.reallocate = UnrealRealloc;
 	UnrealAllocator.zero_allocate = UnrealZeroAlloc;
 	UnrealAllocator.state = nullptr;
 	return UnrealAllocator;
 }

class UnrealMemoryResource : public std::pmr::memory_resource
{
public:
	static UnrealMemoryResource Instance;

private:
	virtual void* do_allocate(std::size_t bytes, std::size_t alignment) override
	{
		return FMemory::Malloc(bytes, alignment);
	}
	
	virtual void do_deallocate(void* p, std::size_t bytes, std::size_t alignment) override
	{
		FMemory::Free(p);
	}
	
	virtual bool do_is_equal(const std::pmr::memory_resource& other) const noexcept override { return true; }
};
