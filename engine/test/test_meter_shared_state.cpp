#include "doctest.h"
#include "MeterSharedState.h"

#include <string>

namespace
{
struct ReadOnlyTelemetryMapping
{
  HANDLE writerHandle = nullptr;
  HANDLE readerHandle = nullptr;
  MeterSharedState* writer = nullptr;
  const MeterSharedState* reader = nullptr;

  ~ReadOnlyTelemetryMapping()
  {
    if (reader) UnmapViewOfFile(reader);
    if (readerHandle) CloseHandle(readerHandle);
    if (writer) UnmapViewOfFile(writer);
    if (writerHandle) CloseHandle(writerHandle);
  }

  bool Open()
  {
    const std::wstring name =
        L"Local\\VAC3TelemetryReadOnlyTest_" +
        std::to_wstring(GetCurrentProcessId()) + L"_" +
        std::to_wstring(GetTickCount64());

    writerHandle = CreateFileMappingW(INVALID_HANDLE_VALUE, nullptr, PAGE_READWRITE,
                                      0, sizeof(MeterSharedState), name.c_str());
    if (!writerHandle) return false;

    writer = static_cast<MeterSharedState*>(
        MapViewOfFile(writerHandle, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(MeterSharedState)));
    if (!writer) return false;

    ZeroMemory(writer, sizeof(MeterSharedState));
    writer->magic = kMeterMagic;
    writer->version = kMeterVersion;
    InterlockedExchange64(&writer->captureDroppedFrames, 1234);
    InterlockedExchange64(&writer->outputUnderrunFrames, 480);
    InterlockedExchange(&writer->encoderQueueBursts, 3);
    InterlockedExchange(&writer->surroundFilterState, 2);

    readerHandle = OpenFileMappingW(FILE_MAP_READ, FALSE, name.c_str());
    if (!readerHandle) return false;

    reader = static_cast<const MeterSharedState*>(
        MapViewOfFile(readerHandle, FILE_MAP_READ, 0, 0, sizeof(MeterSharedState)));
    return reader != nullptr;
  }
};
} // namespace

TEST_CASE("Control Center diagnostics read telemetry from a FILE_MAP_READ view")
{
  ReadOnlyTelemetryMapping mapping;
  REQUIRE(mapping.Open());
  REQUIRE(mapping.reader->magic == kMeterMagic);
  REQUIRE(mapping.reader->version == kMeterVersion);

  CHECK(ReadMeterCounter(&mapping.reader->captureDroppedFrames) == 1234ULL);
  CHECK(ReadMeterCounter(&mapping.reader->outputUnderrunFrames) == 480ULL);
  CHECK(ReadMeterLong(&mapping.reader->encoderQueueBursts) == 3);
  CHECK(ReadMeterLong(&mapping.reader->surroundFilterState) == 2);

  // Confirm the read-only consumer sees producer updates without using an
  // Interlocked read-modify-write operation on its FILE_MAP_READ view.
  InterlockedExchange64(&mapping.writer->captureDroppedFrames, 9876);
  InterlockedExchange(&mapping.writer->encoderQueueBursts, 1);
  CHECK(ReadMeterCounter(&mapping.reader->captureDroppedFrames) == 9876ULL);
  CHECK(ReadMeterLong(&mapping.reader->encoderQueueBursts) == 1);
}
