// Copyright Tempo Simulation, LLC. All Rights Reserved

#pragma once

#include "HAL/UnrealMemory.h"

#include "rcl/allocator.h"
#include "rcutils/allocator.h"

#if PLATFORM_LINUX
#include <experimental/memory_resource>
namespace std::pmr
{
  template <class _ValueT>
  using polymorphic_allocator = std::experimental::pmr::polymorphic_allocator<_ValueT>;
  using memory_resource = std::experimental::pmr::memory_resource;
}
#else
#include <memory_resource>
#endif

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
	static TEMPOROS_API UnrealMemoryResource Instance;

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

inline std::shared_ptr<std::pmr::polymorphic_allocator<void>> GetPolymorphicUnrealAllocator()
{
	return std::make_shared<std::pmr::polymorphic_allocator<void>>(&UnrealMemoryResource::Instance);
}
