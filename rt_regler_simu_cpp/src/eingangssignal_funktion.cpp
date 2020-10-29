// Copyright 2020 DMT Technische Universität Dresden
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <string>
#include <utility>
#include <sched.h> //benötigt für Einstellung sched_richtlinie_prioritaet

#include <stdio.h>
#include <sys/mman.h> // benötigt für mlockall()
#include <unistd.h> // benötigt für sysconf (int name);
#include <malloc.h> // für MemoryAllocator
#include <sys/time.h> // wird für getrusage benötigt
#include <sys/resource.h> // wird für getrusage benötigt
#include <execinfo.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/strategies/allocator_memory_strategy.hpp"
#include "std_msgs/msg/u_int64.hpp"

#include <rclcpp/strategies/message_pool_memory_strategy.hpp> // benötigt für Multi-Message Memory Allocator
#include <rttest/rttest.h> // benötigt für Memory mapping und Vorallocation

#include <inttypes.h> // benötigt für Printf() unit64_t
#include <stdint.h> // benötigt für Printf() unit64_t

#include <chrono> // benötigt um Zeitstampel zuzugreifen
#include <functional>

using namespace std::chrono_literals;

//////////////////////////////////////////////////////////////////////////////
///////////     angepasste  echtzeitfaehige  Memory  Allocator    ////////////
//////////////////////////////////////////////////////////////////////////////

//aufzählen, wie haufig wird angepassten Memory Allocater aufgeruft.
static uint64_t num_allocs = 0;
static uint64_t num_deallocs = 0;

template<typename T = void>
struct MyAllocator
{
public:
  using value_type = T;
  using size_type = std::size_t;
  using pointer = T *;
  using const_pointer = const T *;
  using difference_type = typename std::pointer_traits<pointer>::difference_type;

  MyAllocator() noexcept
  {
  }

  ~MyAllocator() noexcept {}

  template<typename U>
  MyAllocator(const MyAllocator<U> &) noexcept
  {
  }

  T * allocate(size_t size, const void * = 0)
  {
    if (size == 0) {
      return nullptr;
    }
    num_allocs++;
    return static_cast<T *>(std::malloc(size * sizeof(T)));
  }

  void deallocate(T * ptr, size_t size)
  {
    (void)size;
    if (!ptr) {
      return;
    }
    num_deallocs++;
    std::free(ptr);
  }

  template<typename U>
  struct rebind
  {
    typedef MyAllocator<U> other;
  };
};

template<typename T, typename U>
constexpr bool operator==(
  const MyAllocator<T> &,
  const MyAllocator<U> &) noexcept
{
  return true;
}

template<typename T, typename U>
constexpr bool operator!=(
  const MyAllocator<T> &,
  const MyAllocator<U> &) noexcept
{
  return false;
}


static bool beim_Ausfuehren = false;

////aufzählen, wie haufig wird Globale Memory Allocater aufgeruft.
static uint64_t global_runtime_allocs = 0;
static uint64_t global_runtime_deallocs = 0;

void * operator new(std::size_t size)
{
  if (beim_Ausfuehren) {
    global_runtime_allocs++;
  }
  return std::malloc(size);
}

void operator delete(void * ptr, size_t size) noexcept
{
  (void)size;
  if (ptr != nullptr) {
    if (beim_Ausfuehren) {
      global_runtime_deallocs++;
    }
    std::free(ptr);
  }
}

void operator delete(void * ptr) noexcept
{
  if (ptr != nullptr) {
    if (beim_Ausfuehren) {
      global_runtime_deallocs++;
    }
    std::free(ptr);
  }
}

///////////////////////////////////////////////////////////////////////////////
/////////////     Scheduler auswählen   Priorität einstellen   ////////////////
///////////////////////////////////////////////////////////////////////////////

int sched_richtlinie_prioritaet (size_t sched_prioritaet, int richtlinie)
{
  struct sched_param param;

  param.sched_priority = sched_prioritaet;

  // note that sched_setscheduler can set the priority of an arbitrary process
  return sched_setscheduler(0, richtlinie, &param);
}
///////////////////////////////////////////////////////////////////////////////
/////////////////////////   Memory Vorallocation   ////////////////////////////
///////////////////////////////////////////////////////////////////////////////

/// Declare a function pointer into which we will store the default malloc.
static void * (* prev_malloc_hook)(size_t, const void *);

// Use pragma to ignore a warning for using __malloc_hook, which is deprecated (but still awesome).
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
/// Implement a custom malloc.
/**
 * Our custom malloc backtraces to find the address of the function that called malloc and formats
 * the line as a string (if the code was compiled with debug symbols.
 * \param[in] size Requested malloc size.
 * \param[in] caller pointer to the caller of this function (unused).
 * \return Pointer to the allocated memory
 */
static void * testing_malloc(size_t size, const void * caller)
{
  (void)caller;
  // Set the malloc implementation to the default malloc hook so that we can call it implicitly
  // to initialize a string, otherwise this function will loop infinitely.
  __malloc_hook = prev_malloc_hook;

  if (beim_Ausfuehren) {
    fprintf(stderr, "Called malloc during realtime execution phase!\n");
    rclcpp::shutdown();
    exit(-1);
  }

  // Execute the requested malloc.
  void * mem = malloc(size);
  // Set the malloc hook back to this function, so that we can intercept future mallocs.
  __malloc_hook = testing_malloc;
  return mem;
}

/// Function to be called when the malloc hook is initialized.
void init_malloc_hook()
{
  // Store the default malloc.
  prev_malloc_hook = __malloc_hook;
  // Set our custom malloc to the malloc hook.
  __malloc_hook = testing_malloc;
}
#pragma GCC diagnostic pop

/// Set the hook for malloc initialize so that init_malloc_hook gets called.
void(*volatile __malloc_initialize_hook)(void) = init_malloc_hook;


///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////


int main(int argc, char ** argv)
{
  //////////////////////////////////////////////////////////////////////////
  /////////////////////////    Inizialization    ///////////////////////////
  //////////////////////////////////////////////////////////////////////////
  using rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy;
  using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;
  using Alloc = MyAllocator<void>;
  using MessageAllocTraits =
    rclcpp::allocator::AllocRebind<std_msgs::msg::UInt64, Alloc>;
  using MessageAlloc = MessageAllocTraits::allocator_type;
  using MessageDeleter = rclcpp::allocator::Deleter<MessageAlloc, std_msgs::msg::UInt64>;
  using MessageUniquePtr = std::unique_ptr<std_msgs::msg::UInt64, MessageDeleter>;
  // Übergebe die Eingabeargumente an rttest.
  // rttest speichert relevante Parameter und weist Puffer für die Datenerfassung zu
  rttest_read_args(argc, argv);
  // Übergebe die Eingabeargumente an rclcpp und initialisiere den IMU-Publisher.
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr Eingangssignal;//= std::make_shared<Eingangssignal>();
  ///////////////////////////////////////////////////////////////////////////
  //Instanziieren die Node später, da einige wichtige Argumenten hier
  //abgelesen werden muss.
  ///////////////////////////////////////////////////////////////////////////

  ////////////////////////////////////////////////////////////////////////////
  ///////////////   führt den Task im Intra_process Pipeline    //////////////
  ////////////////////////////////////////////////////////////////////////////
    printf("Intra-prozess Pipeline Status: An\n");
    auto context = rclcpp::contexts::default_context::get_global_default_context();
    auto options = rclcpp::NodeOptions()
      .context(context)
      .use_intra_process_comms(true);

  ////////////////////////////////////////////////////////////////////////////
  //////////////////////////////     Node Einstellen    //////////////////////
  ////////////////////////////////////////////////////////////////////////////

  //Eingangssignal x_Ist als eine Node, deren Information mit einer Frequenz von 200Hz aktualisiert.
  Eingangssignal = rclcpp::Node::make_shared("IMU_Signal", options);


   /////////////////////////////////////////////////////////////////////////////
   //////////////     QoS Einstellung auf die beste Echtzeitleistung    /////////
   //////////////////////////////////////////////////////////////////////////////
 //  auto qos = rclcpp::QoS(
    // Die Verlaufseinstellung "KEEP_LAST" weist DDS an,
    // vor dem Senden einen Wertepuffer mit fester Größe zu speichern, um die Wiederherstellung
    // bei verworfenen Nachrichten zu erleichtern. "depth" gibt die Größe dieses Puffers an.
    // In diesem Beispiel werden die Leistung und die eingeschränkte Ressourcennutzung
    // (Verhinderung der Pagefaults) anstelle von Zuverlässigkeit.
    // Daher setzen wir die Größe des Verlaufspuffers auf 1.
//       rclcpp::KeepLast(1)
//     );
//  qos.best_effort();

  uint64_t counter = 0;
  auto callback = [&counter](std_msgs::msg::UInt64::SharedPtr msg) -> void
    {
      (void)msg;
      ++counter;
    };
  // Erstelle einen benutzerdefinierten Allokator und übergeben Sie den Allocator
  // an den Publisher
  auto alloc = std::make_shared<Alloc>();
  rclcpp::PublisherOptionsWithAllocator<Alloc> publisher_options_IMU;
  publisher_options_IMU.allocator = alloc;
  // Vom IMU wird x_Ist Published
  auto IMU_publisher = Eingangssignal->create_publisher<std_msgs::msg::UInt64>(
    "x_Ist", 5, publisher_options_IMU);
  rclcpp::SubscriptionOptionsWithAllocator<Alloc> subscription_options_IMU;
  subscription_options_IMU.allocator = alloc;
  auto msg_mem_strat = std::make_shared<
    rclcpp::message_memory_strategy::MessageMemoryStrategy<
        std_msgs::msg::UInt64, Alloc>>(alloc);
  auto IMU_selfsubscriber = Eingangssignal->create_subscription<std_msgs::msg::UInt64>(
    "x_Ist", 5, callback, subscription_options_IMU, msg_mem_strat);

  // Erstell eine MemoryStrategy, die die vom Executor während des
  // Ausführungspfad und füge die MemoryStrategy in den Executor ein.
  rclcpp::executor::ExecutorArgs args;
  std::shared_ptr<rclcpp::memory_strategy::MemoryStrategy> memory_strategy =
    std::make_shared<AllocatorMemoryStrategy<Alloc>>(alloc);
  args.memory_strategy = memory_strategy;

  rclcpp::executors::SingleThreadedExecutor executor(args);

  // Füge die Knoten zum Executor hinzu.
  executor.add_node(Eingangssignal);

  MessageDeleter message_deleter;
  MessageAlloc message_alloc = *alloc;
  rclcpp::allocator::set_allocator_for_deleter(&message_deleter, &message_alloc);
  rclcpp::sleep_for(std::chrono::milliseconds(5));

  //Zeitstampel Message für Test.
  uint64_t time_now;
  //Echtzeitsichere Scheduler Richtlinie (round robin)
  if(!sched_richtlinie_prioritaet(97, SCHED_RR)){
      perror("Scheduler Richtlinie: SCHED_RR \n höchste Priorität eingestellt.");
  }
  // Sperren den aktuell zwischengespeicherten virtuellen Speicher im RAM
  // sowie alle zukünftigen Speicherzuordnungen.
  // und tun unser Bestes, um den gesperrten Speicher vorzugeben,
  // um zukünftige Pagefaults zu vermeiden.
  // Wird mit einem Fehlercode ungleich Null zurückgegeben,
  // wenn ein Fehler aufgetreten ist (unzureichende Ressourcen oder Berechtigungen).
  // Tue dies immer als letzten Schritt der Initialisierungsphase.
  // Weitere Informationen finden Sie unter rttest / rttest.cpp.
  //if (rttest_lock_and_prefault_dynamic() != 0) {
  // fprintf(stderr, "Nicht alle Virtuelle Memory wird gesperrt.\n");
  //}
  ////////////////////////////////////////////////////////<//////////////////
  ////////////////////   Ende der Inizialization   /////////////////////////
  //////////////////////////////////////////////////////////////////////////
  beim_Ausfuehren = true;
  ////////////////////////////////////////////////////////////////////////////
  //////////////////////    DER Echtzeit-sichere Teil    /////////////////////
  ////////////////////////////////////////////////////////////////////////////
  while (rclcpp::ok()) {
    // Create a message with the custom allocator, so that when the Executor deallocates the
    // message on the execution path, it will use the custom deallocate.
    auto ptr = MessageAllocTraits::allocate(message_alloc, 1);
    MessageAllocTraits::construct(message_alloc, ptr);
    MessageUniquePtr msg_p(ptr, message_deleter);
    //Momentane Zeitstempel wird abgeschickt.
    time_now = rclcpp::Clock().now().nanoseconds();
    msg_p->data = time_now;
    IMU_publisher->publish(std::move(msg_p));
    rclcpp::sleep_for(std::chrono::milliseconds(5));
//    printf("Hallo world");
    executor.spin_some();
  }
  ////////////////////////////////////////////////////////////////////////////
  ////////////////////    Ende des Echtzeit-sichere Teils    /////////////////
  ////////////////////////////////////////////////////////////////////////////
  beim_Ausfuehren = false;

  // Am Ende wird die Häufigkeit des Abruf der standard - Memoryallocator und
  printf("Global new was called %" PRIu64 " times during spin\n", global_runtime_allocs);
  // die Häufigkeit des Abruf der benutzerdefinierte Memoryallocator
  printf("Custom Allocator new was called %" PRIu64 " times during spin\n", num_allocs);
  // angezeigt.
  return 0;
}
