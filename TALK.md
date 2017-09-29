# Talk description

> Fast, efficient, safe: pick three.
>
> Embedded systems are highly concurrent; they must respond to several different external stimuli
> within tight time constraints while maintaining a low power consumption. At the core of these
> systems we usually find microcontrollers, low end Systems on a Chip (SoCs) with just about enough
> resources to fulfill their tasks: tens of MHz of CPU frequency and a few KBs of RAM are usual.

> In this talk we’ll explore how to do efficient multitasking on these systems using zero cost,
> memory safe abstractions.

# Agenda

- Microcontroller basics

- Cooperative multitasking

- Preemption

- Locks, locks, locks

- Fearless concurrency

- What's next?

> This is the agenda for the talk. We'll first cover microcontroller basics. Then we'll look into
> cooperative multitasking. Then into preemption as a way to achieve multitasking and the problem
> that come from using preemption. Next come locks as a solution to those problems. Then I'll
> present a fearless concurrency framework and conclude with future work in this space.

# Microcontrollers

I/O with the world.

IMAGE

Cheap. Simple. Low power. Full control.

> Microntrollers

> Single core devices with limited amounts of memory and processor power. Why would you want to use
> them in the first place?

> The main reason is being able to do I/O with the world

NEXT

> On a general purpose computer I/O means files, sockets and databases.

> On a microcontroller I/O means turning lights on or off, controlling motors and reading sensors.

> Some of you may be wondering: Why not use a SBC, like the RPi, instead? It can do the same of I/O
> and runs Linux.

NEXT

> They are cheaper, and cheap is good for business.

NEXT

> They are simpler systems. Simple enough that you can program them from scratch.

NEXT

> They have lower power consumption, which is good for battery life.

NEXT

> And you have full control over how tasks get scheduled when you are not using an OS. This can be
> *vital* to meet the timing requirements of an application. This is why microcontrollers are
> commonly used in real time systems. In this talk I'll focus on bare metal systems, which means no
> underlying OS.

# A minimal program

``` rust
#![no_std]

extern crate cortex_m_rt;
extern crate stm32f103xx;

// before: device boots and RAM is initialized

fn main() {
    // do stuff here
}

// after: CPU goes to sleep
```

> Here's a minimal microcontroller program.

> That uses the [quickstart] framework.

> It looks very similar to any other Rust program.

> There's this extra #![no_std] attribute at the top, which means no standard library.

> Then we have some external crates that take care of booting the device.

> Then you have your standard main function

> Important to note here: on bare metal systems, where there's no OS, only a single program can run
> at any time. And this program must never end.

> So what happens here when the main function returns? The quickstart runtime will put the
> microcontroller to sleep.

> This framework supports both Cortex-M and MSP430 microcontrollers so you can write programs the
> same. And someone is already porting this framework to RISC-V.

# I/O

``` rust
fn main() {
    // omitted: initialization code

    let register = 0x4001_1010 as *mut u32;

    unsafe {
        // turns an LED on
        ptr::write_volatile(register, 1 << 29);

        // turns the LED off
        ptr::write_volatile(register, 1 << 13);
    }
}
```

> Next: How do we do I/O on a microcontroller?

> Basically, we just have read or write to memory. To a special region of memory called peripheral
> memory.

> This program does two write operations. The first one turns an LED on; the second one turns it
> off.

> This approach to I/O is called memory mapped I/O.

> Because writing to peripheral memory is side effectful and because the environment can change the
> state of peripheral memory at any time all operations on peripheral memory must be volatile.
> Otherwise the compiler will misoptimize your program.

# 3 common peripherals

> Let's look at 3 common peripherals that I'll use throughout this presentation.

## GPIO

``` rust
fn main() {
    // omitted: initialization

    LED.on();
    LED.off();
}
```

General Purpose I/O

> First one: general purpose I/O

> This peripheral lets you output digital signals on the microcontroller pins. With proper
> electronics these signals can turn things on or off, or in general they can toggle some binary
> state, for example you can use this peripheral to lock / unlock a door.

> That's for the output part of the peripheral. The input part lets you read some external binary
> state which could map to "is this button pressed?" or "is the temperature above or below some level?".

> In this presentation I'll use this high level LED abstraction that handles memory writes under
> the hood and gives us an API to turn an LED on or off.

## Timers

``` rust
fn main() {
    // omitted: more initialization

    timer.init(500.ms(), rcc); // periodic
    timer.resume(); // start alarm

    loop {
        timer.bwait();
        LED.on();
        timer.bwait();
        LED.off();
    }
}
```

> Next: Timers.

> Timers are useful to do periodic tasks and also to measure the duration of external events.

> In this presentation I'll use them as a periodic alarm mechanism.

> This program sets an alarm to go off every 500 milliseconds, and the alarm is used to blink an
> LED. Every time the alarm goes off we'll toggle the state of the LED.

> This `bwait` method *blocks* the program until the alarm goes off giving us the right delays.

## Timers

How does the `bwait` method work?

``` rust
// simplified
fn bwait() {
    loop {
        // did the alarm go off?
        if tim3.sr.read().uif().bit_is_clear() {
            // no, ask again
        } else {
            // yes, clear the bit
            tim3.sr.modify(|_, w| w.uif().clear_bit())
            return;
        }
    }
}
```

*Busy waiting*

> We can check the status of timer by reading its status register. This status register is a 32 bit
> piece of peripheral memory. One of the bits in this register indicates whether the alarm has gone
> off or not yet. When the alarm goes off the corresponding bit is set to 1.

> To wait for the alarm we *continuously* poll the status register until the bit value becomes 1.
> This approach is known as busy waiting.

NEXT

>It's quite wasteful in terms of CPU cycles, but the program behaves as expected.

## Serial

"Serial port" communication

``` rust
fn main() {
    // omitted: more initialization

    serial.bwrite_all(b"Hello!\n").unwrap();

    loop {
        let byte = serial.bread().unwrap();
        serial.bwrite(byte).unwrap();
    }
}
```

> Next. Serial which stands for serial port communication.

> Serial is a simple communication interface. It's used to exchange information between two end
> points. Each end point can send data to the other side at any time.

> This program starts by sending out a greeting to the other end. Then it waits for input. Each byte
> received is sent back to the sender. All the operations here are blocking.

## Cooperative multitasking

Generators.

> Let's move onto concurrency.

> Let's say we want to merge the last two programs into a single program that runs the two tasks
> concurrently. How can we achieve this multitasking? One way is to do cooperative multitasking
> using

NEXT

> generators.

### Generators

``` rust
// simplified
fn gread() -> impl Generator<Return=u8, Yield=()> {
    || {
        // while no new data available
        while usart1.sr.read().rxne().bit_is_clear() {
            yield;
        }

        // read byte
        usart1.dr.read().as_bits()
    }
}
```

> Apart from the blocking API available on the serial and timer abstractions. There's also a
> non blocking API based on generators.

> Here's an example of reading a single byte from the serial interface using generators.

> In this generator API instead of busy waiting for a condition we *yield* control back to the
> caller while the condition is not met.

### Tasks

> Now we can rewrite the blocking programs as tasks using generators.

> Each task is an infinite generator.

> The first one is the echo program.

> Logic is exactly the same: read a byte and send it back. But this time we use the generator API.
> Instead of blocking on the generator we use the `await!` macro to drive the generators to
> completion. The trick here is that the `await!` macro never blocks; instead, it yields control
> back to the caller when the generator can't make more progress.

NEXT
NEXT

> In the second task, the blinking LED task, we change the logic a bit just to show that generators
> can capture variables allocated on the stack. Here the `on` variable tracks the state of the LED.
> Here we also use the `await!` macro to non-blockingly wait for the alarm.

### Concurrent execution

> Now let's put everything together. We create the tasks which are infinite generators and then go
> into an infinite loop that will resume each generator in turn. The result is that the processor
> will execute a bit of a task until it can't make any more progress, then it will switch to the
> next task, and so on and so forth.

NEXT

> Note that this program *still* has busy waiting. When neither task can resume execution the
> processor will continuously poll status registers.

## The need for preemption

> Now, imagine this scenario: You need to execute two periodic tasks concurrently, like in the
> previous program.

NEXT

> This time: task 1 needs to be run every 10 ms and takes 2 ms to complete.

NEXT

> And task 2 needs to be run every 1 ms and takes 100 us to complete.

NEXT

> Can these timing constrains be satisfied using cooperative multitasking?

> Let's analyze this.

> Let's say we begin executing task 1. Task 1 is executed for 1 ms and we get a request to execute
> task 2. But if task 1 never yields then it will be another millisecond until task 2 gets a chance
> to run. This means that task 2 missed its deadline twice. In the 2 ms that took executing task 1
> task 2 was supposed to be executed twice but it never got a chance to run.

> In a hard real time system missing even a *single* deadline means system failure.

NEXT

> So in general, no. With cooperative multitasking you can hardly make any claim about meeting
> timing requirements.

> With preemption this example could have worked. If task 2 could have preempted task 1 instead of
> waiting for it to yield control then it could have been serviced at the right time.

### Interrupts

A callback mechanism provided by the hardware.

``` rust
#[macro_use(interrupt)]
extern crate stm32f103xx;

fn main() {
    // ..
}

// trigger: user pressed a button
interrupt!(EXTI0, handler);

// callback
fn handler() {
    // ..
}
```

> Thankfully microcontrollers provide a hardware mechanism for preemption: interrupts.

> How do they work? There are several interrupts; each one is triggered by a different event. You
> can register *one* interrupt handler, which is just a function, for each interrupt. When the
> interrupt event arrives the hardware calls your interrupt handler.

> In this example we've registered a handler for the EXTI0 interrupt, whose trigger is "the user
> pressed a button". Normally we'll be executing the `main` function. But at some point the user
> will press the button. Then the processor will halt the execution of `main` to execute the
> interrupt handler. Once the interrupt handler returns it will resume the execution of main.

> So basically the interrupt handler can preempt the main function. You can also think of this as
> interrupts having higher priority than main.

### Interrupts: echo

Event driven version.

``` rust
fn main() {
    // omitted: initialization
    serial.listen(Event::Rxne); // new data available

    loop { asm::wfi(); } // sleep
}

interrupt!(USART1, handler);

fn handler() {
    // omitted: constructing `serial`
    let byte = serial.read().unwrap();
    serial.write(byte).unwrap();
}
```

*No busy waiting*

> To get a better idea of the power of interrupts we can rewrite the echo program from before using
> interrupts. This time all the echo logic is performed in the interrupt handler.

> This leaves nothing to do in the main loop so we just send the microcontroller to sleep.

> This version has no busy waiting.

NEXT

> When there's no data to process the microcontroller goes to sleep saving energy.

### Interrupts: Memory sharing

Do NOT do this.

``` rust
static mut DATA: Data = /* .. */;

fn main() {
    unsafe { /* do something with DATA */ }
}

interrupt!(EXTI0, handler);

fn handler() {
    unsafe { /* do something with DATA */ }
}
```

> At some point you'll want to share state between the main function and an interrupt handler or
> between interrupts.

> The only way to do this is using a statically allocated variable.

> And the first thing you'll probably try is using a `static mut` variable. Don't do that. Access to
> `static mut` variables is unsynchronized which opens to door to data races and race conditions.

## Locks, locks, locks

> So *how* can we share data safely? One way is to use locks to synchronize access.

### Disabling interrupts

This is actually *memory safe*.

``` rust
use cortex_m::interrupt;

static mut DATA: Data = /* .. */;

fn main() {
    loop {
        interrupt::free(|_| unsafe {
            DATA.modify();
        });
    }
}

interrupt!(EXTI0, handler);

fn handler() {
    unsafe { DATA.modify() }
}
```

*Not all contexts need to lock the data*

> Let's look at this program from before. What can go wrong?

> The problematic scenario here is the following: main is modifying the data and it gets preempted
> by the interrupt handler midway the process. In this scenario the interrupt handler may encounter
> the data to be in an inconsistent state or even find corrupted data.

> To prevent this problem we need to synchronize the concurrent operations on the data so that
> one happens before *or* after the other. The order is not important as long as one operation
> doesn't occur midway the other.

> To achieve the required synchronization we can disable the interrupts while main is modifying the
> data. With this change the interrupt handler can only fire before or after main modifies the data.

NEXT

NEXT

NEXT

NEXT

NEXT

> Then we have the operations synchronized.

NEXT

> Note that, unlike mutexes, not all the contexts of execution need to lock the data.

### More interrupts

Is this memory safe?

``` rust
fn main() {
    loop {
        interrupt::free(|_| unsafe { DATA.modify(); });
    }
}

interrupt!(EXTI0, exti0);
fn exti0() {
    unsafe { DATA.modify() }
}

interrupt!(EXTI1, exti1);
fn exti1() {
    unsafe { DATA.modify() }
}
```

*It depends*

> What happens if we add another interrupt that also contends for the data.

> Is this program memory safe? Both interrupts are accessing the data without locking.

> The answer is: it depends on the architecture.

NEXT

> On MSP430 interrupt nesting is disabled by default so an interrupt can not preempt another
> interrupt. In that case this program is memory safe.

> OTOH, on Cortex-M you can assign priorities to interrupts and higher priority interrupts can
> preempt lower priority ones. So we don't have enough information to say whether this program is
> memory safe on a Cortex-M device.

### Nested interrupts (Cortex-M)

This is OK.

``` rust
// priority = 1
fn exti0() {
    interrupt::free(|_| unsafe { DATA.modify(); });
}

// priority = 2
fn exti1() {
    interrupt::free(|_| unsafe { DATA.modify(); });
}

// priority = 3
fn exti2() {
    unsafe { DATA.modify() }
}
```

> But if we know the priorities of all the interrupts then we can answer the question.

> This version of the program is memory safe. The highest priority interrupt can access the data
> without a lock; all the other interrupt need a lock to access the data.

### Better locks (Cortex-M)

`exti0` needs to block `exti1` for memory safety

``` rust
// priority = 1
fn exti0() {
    interrupt::free(|_| unsafe { DATA.modify(); });
}

// priority = 2
fn exti1() {
    unsafe { DATA.modify(); };
}

// priority = 3
fn exti2() {
    // doesn't use DATA but gets blocked
}
```
... but `exti2` doesn't need to be blocked!

> What about this scenario? Again three interrupts operating at different priorities. But only two
> of them, the ones with the lowest priorities, contend for the same data.

> Using our disable interrupts lock in the task with lowest priority, `exti0`, will also prevent the
> highest priority task, `exti2`, from starting.

> This is unnecessary

NEXT

> because `exti0` only needs to block `exti1` for memory safety.

> Can we do better? This extra blocking is not good from an scheduling perspective. High priority
> tasks should *not* be blocked by low priority unless it's a must.

### Better locks (Cortex-M3+)

``` rust
// simplified
fn raise<R, F>(new_priority: u8, f: F) -> R
where
    F: FnOnce() -> R,
{
    let old_priority = basepri::read();
    basepri_max::write(new_priority); // NOTE can't lower the priority
    let r = f();
    basepri::write(old_priority); // restore
    r
}
```

> Here's a different locking mechanism: instead of disabling *all* the interrupts we can temporarily
> raise the priority of the current execution context to block *some* interrupts from starting.

> This mechanism shown here is for Cortex-M3 and up; this doesn't work for MSP430 or Cortex-M0
> devices.

### Better locks (Cortex-M3+)

``` rust
// priority = 1
fn exti0() {
    // raises priority to 2
    raise(2, |_| unsafe { DATA.modify(); });
}

// priority = 2
fn exti1() {
    unsafe { DATA.modify(); };
}

// priority = 3
fn exti2() {
    // doesn't use DATA
}
```

> If we apply this mechanism to the previous program we can have enough interrupt blocking to
> achieve memory safety without sacrificing schedulability. In this new version the raised priority
> of `exti0` blocks `exti1` but not `exti2`.

### Immediate Ceiling Priority Policy (ICPP)

- "raised priority value" = Ceiling Priority

- "Each resource is assigned a priority ceiling, which is a priority equal to the
  highest priority of any interrupt which may lock the resource."

- Guarantees deadlock free execution if correctly used.

> How do we pick the priority value to pass to the `raise` function?

> Luckily for us, this scheduling problem has been studied since the 80s and there's an
> answer: the Immediate Priority Ceiling Protocol. The title links to a paper that analyzes this
> scheduling policy if you want to read that later.

NEXT

NEXT

NEXT

## Fearless concurrency

> Now you have some idea of how to properly use locks to achieve memory safety. Still manually
> applying the rules we've seen is error prone and directly using `static mut` variables is
> `unsafe`. Instead, we'd like to only write safe code and have the compiler enforce memory safety.
> That's why we have created the Real Time For the Masses framework.

NEXT

Also known as the RTFM framework

NEXT

> This framework is actually a Rust port of the Real Time For the Masses *language* that was
> developed at LTU some time ago.

### RTFM

<ul>
  <li class="fragment"> Model: Tasks and resources </li>
  <li class="fragment"> Task = Interrupt
    <ul>
      <li class="fragment"> The hardware will schedule the tasks </li>
    </ul>
  </li>
  <li class="fragment"> Resource: safe mechanism to share memory
    <ul>
      <li class="fragment"> Ceilings are computed automatically </li>
      <li class="fragment"> PCP: Deadlock freedom </li>
    </ul>
  </li>
  <li class="fragment"> Can still do cooperative multitasking </li>
  <li class="fragment"> Downside: requires whole program analysis </li>
</ul>

NEXT

> The RTFM concurrency model is based on tasks and resources.

NEXT

> One task in this model maps to one interrupt.

> With this approach we can make the hardware schedule the tasks for us; this
> makes scheduling very efficient as we don't need any bookkeeping in the software.

> A resource is a zero cost mechanism for sharing data, or peripherals, between tasks.

> The framework uses ICPP so we have deadlock free execution enforced at compile time. Note that
> this is a stronger property that the data race freedom you get from using Rust. In Rust deadlocks
> are considered safe.

NEXT

> The ceiling required for ICPP are computed automatically by the framework.

NEXT

> Even though the task model favors event driven preemptive multitasking cooperative multitasking is
> also supported.

NEXT

> In exchange for all these nice properties the framework requires whole program analysis to work
> but this isn't a showstopper.

### `app!`

`app!` is the specification of the system.

> Let's take a look at a RTFM application.

> All RTFM applications include an `app!` macro which provides a description of the system to the
> RTFM framework. The main two components of the specification are a list of resources and a list of
> tasks. Resources are specified as plain static variables. Each task entry binds an interrupt to a
> task and must specify the resources that the task has access to as well as the task priority.

> This macro is where we pay for the whole program analysis downside. All tasks and resources must
> be defined in this macro and there can only be an instance of this macro and that instance must be
> in the root of the application crate. IOW, you can't split your task and resource definitions
> across different crates.

> Having everything in a single place lets us perform whole program analysis with existing Rust
> language features. We can also specify peripherals as resources.

> In this example we have two resources and two tasks. One resource is owned by a single task and
> the other one is shared between tasks. The tasks run at different priorities.

### `init` and `idle`

> The `app!` macro will expand into the main function. So the user doesn't have to provide one;
> instead, they have to supply these `init` and `idle` functions.

> `init` will be called first and runs with interrupts disabled. This function has access to all
> peripherals. System initialization and configuration should go in this function.

> After `init` returns, interrupts are re-enabled and `idle` is executed.

> `idle` is a never ending function that runs after `init`.  `idle` semantically has a priority of
> 0, the lowest priority. And it's where you should put your cooperative multitasking logic.

### Tasks

> Tasks are provided by the user as functions. The framework will choose the task signature
> according to the specification of the application.

> These `Resources` structures you see in the signatures are generated by the framework with the
> goal of

NEXT

NEXT

> In this example you can see that only the low priority task, `exti0`, needs to lock the shared
> resource to modify it.

### Beyond locking

RTFM also wants to support lock free data structures.

- Support for atomics with better scoping.

- Single producer single consumer ring buffer
  - Statically allocated, interrupt safe and lock free

> Locks are nice because they always work so they are a good default but RTFM also wants to support
> lock free data structures in 100% safe code.

> atomics: It's memory safe to place an atomic in a `static` variable and it's memory safe to modify
> it from any part of the program. But a global scope diminishes the correctness of the atomic
> because then any task can use it.

> Another use case we are looking into is a single producer single consumer ring buffer. It requires
> that only one task can put items into the buffer and only one can take them from it.

## What's next?

### Concurrent Reactive Objects

> To my knowledge, RTFM is the most efficient way to do preemptive multitasking but it's also mainly
> a toolbox of concurrency primitives. And the only way for inter-task communication it provides is
> memory sharing which can get hard to reason about in large programs.

NEXT

> The next extension to the RTFM framework are Concurrent Reactive Objects or CROs.

### cro.png

> Model: Objects and Message Passing

> Here we have three objects: USART, STM and DMA. The last two objects are encapsulated into an LED
> component.

> Synchronous and asynchronous messages

> Timing semantics

> Interrupts are triggers
