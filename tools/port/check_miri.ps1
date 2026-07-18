param(
    [switch]$All
)

$ErrorActionPreference = "Stop"

function Invoke-MiriTest {
    param([string[]]$Arguments)
    & cargo +nightly miri test @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "Miri failed: cargo +nightly miri test $($Arguments -join ' ')"
    }
}

# The full fixed-container library covers vectors, maps, intrusive lists,
# object pools, endian helpers, and iterators. The focused queue suites cover
# the shared-memory adapters used by interrupt/main-loop handoff.
Invoke-MiriTest @("-p", "bsw-estd", "--lib")
Invoke-MiriTest @("-p", "bsw-util", "--lib", "spsc::tests")
Invoke-MiriTest @("-p", "bsw-io", "memory_queue")
Invoke-MiriTest @("-p", "bsw-bsp-stm32", "--lib", "can_isr::tests")

if ($All) {
    Invoke-MiriTest @("-p", "bsw-util", "--lib")
    Invoke-MiriTest @("-p", "bsw-io", "--lib")
}
