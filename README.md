თბილისის თავისუფალი უნივერსიტეტი

კომპიუტერული მეცნიერებების, მათემატიკისა და ინჟინერიის სკოლა (MACS\[E\])

ელექტრო და კომპიუტერული ინჟინერიის პროგრამა

ალექსანდრე სულაბერიძე

ჯუნიორ პროექტი

ავტომატური დამიზნების სისტემა

ხელმძღვანელი: სულაბერიძე ზვიადი

თბილისი

2024

# 2. ანოტაცია/Annotation {#ანოტაციაannotation}

The main idea of this project was to create a target tracking system,
which based on the data received from a video camera, would detect a
target object and using a mechanical \"Pan-Tilt\" device, with a mounted
end effector (a laser), would aim at the detected object in real-time.
The system should ensure that the object remains in the target.

The relevance of the topic stems from the growing demand for automation
in society. The presented system is flexible, and with the selection of
the right end effector, it is possible to use it for various purposes.

**Possible applications of the system:**

- **Tracking video camera system for security purposes:** The system can
  be used to respond to suspicious or dangerous activities. The camera
  detects the target, while the Pan-Tilt device ensures tracking and
  focusing on it. It is also possible to zoom in on the image to see
  details. This system is useful for both public safety and protection
  of private property and buildings.

- **Automated defense system:** With the selection of special end
  effectors, the system can detect and respond to various threats. For
  example, it can be used to target the source of danger, issue
  warnings, neutralize it, and more. Such a system could be useful for
  protecting private property as well as other strategic facilities.

- **Precision surgery and medical applications:** Due to the fact that
  even the slightest mistakes in the medical field can be fatal, there
  is an increasing demand for systems that can perform certain tasks
  flawlessly without human intervention. With the right end effector
  selection, such a system could be implemented here as well.

- **Industrial applications:** The system is useful for automating
  manufacturing and various processes. For example, it can identify
  objects, perform targeted processing or additional operations as they
  move along a conveyor belt. It is also possible to detect and
  eliminate undesirable activities (e.g., by selecting a robotic arm as
  the end effector to perform these tasks).

The modularity of the system and the ability to use different end
effectors make it quite flexible and provide a wide range of
applications in security, defense, medical, and industrial fields.

My main goal was to create a working prototype of this system, which
could recognize simple objects, people, and others, and target them.

The result of approximately 6 months of work is the pilot version of the
system, which using machine learning algorithms can provide coordinates
to a microcontroller-controlled system, which then directs the end
effector accordingly.

The next step, of course, is to extend the project to the aforementioned
areas and adapt it to specific problems that arise in them.

ამ პროექტის მთავარი იდეა იყო სამიზნე მიმდევარი სისტემის შექმნა, რომელიც
ვიდეოკამერიდან მიღებული მონაცემების საფუძველზე გამოავლენდა სამიზნე
ობიექტს და მექანიკური \"Pan-Tilt\" მოწყობილობის გამოყენებით, მასზე
დამონტაჟებული საბოლოო სამუშაო ორგანოს (End Effector) - ლაზერის
საშუალებით, რეალურ დროში დაუმიზნებდა აღმოჩენილ ობიექტს. სისტემამ უნდა
უზრუნველყოს ობიექტის სამიზნეში დარჩენა.

თემის აქტუალობა გამომდინარეობს საზოგადოებაში ავტომატიზაციის მზარდი
მოთხოვნიდან. წარმოდგენილი სისტემა მოქნილია და სწორი საბოლოო სამუშაო
ორგანოს (End Effector) შერჩევით, შესაძლებელია მისი გამოყენება სხვადასხვა
დანიშნულებით.

**სისტემის შესაძლო გამოყენების სფეროებია:**

- **მიმდევარი ვიდეოკამერის სისტემა უსაფრთხოების მიზნებისთვის:** სისტემა
  შეიძლება გამოვიყენოთ საეჭვო ან საშიშ აქტივობაზე რეაგირებისთვის. კამერა
  აღმოაჩენს სამიზნეს, ხოლო Pan-Tilt მოწყობილობა უზრუნველყოფს მის
  კვალდაკვალ გაყოლას და ფოკუსირებას. აგრეთვე, შესაძლებელია გამოსახულების
  გადიდება დეტალების დასანახად. ეს სისტემა გამოსადეგია როგორც
  საზოგადოებრივი უსაფრთხოების, ასევე კერძო საკუთრებისა და შენობების
  დაცვის მიზნებისთვის.

- **ავტომატიზებული თავდაცვის სისტემა:** სპეციალური საბოლოო სამუშაო
  ორგანოების შერჩევით, სისტემას შეუძლია გამოავლინოს და რეაგირება
  მოახდინოს სხვადასხვა საფრთხეზე. მაგალითად, შეიძლება გამოვიყენოთ
  საფრთხის წყაროს მიზანში ამოღებისთვის, გაფრთხილებისთვის,
  განეიტრალებისთვის და სხვა. Მსგავსი სისტემა შეიძლება გამოსადეგი იყოს
  როგორც კერძო საკუთრების, ისე სხვა სტრატეგიული ობიექტების დაცვაში.

- **პრეციზიული ქირურგია და სამედიცინო გამოყენებები:** იმის გამო რომ
  სამედიცინო სფეროში უმცირესი შეცდომების დაშვებაც კი შეიძლება ფატალური
  იყოს, გაზრდილია მოთხოვნა სისტემებზე, რომლებსაც ადამიანების ჩარევის
  გარეშე, უშეცდომოდ შეუძლიათ გარკვეული დავალებების შესრულება. სწორი End
  Effector-ის არჩევით, აქაც შესაძლოა მსგავსი სისტემის დანერგვა.

- **სამრეწველო გამოყენებები:** სისტემა გამოსადეგია წარმოებისა და
  სხვადასხვა პროცესების ავტომატიზაციისთვის. მაგალითად, მას შეუძლია
  ობიექტების იდენტიფიცირება, მიზნობრივი დამუშავება ან დამატებითი
  ოპერაციების შესრულება კონვეიერზე გადაადგილებისას. აგრეთვე,
  შესაძლებელია არასასურველი აქტივობის აღმოჩენა და აღმოფხვრა (მაგალითად,
  End Effector-ად მექანიზებული ხელის არჩევით, რომელიც ამ დავალებებს
  შეასრულებს)

სისტემის მოდულარულობა და სხვადასხვა სამუშაო ორგანოების გამოყენების
შესაძლებლობა მას საკმაოდ მოქნილს ხდის და იძლევა ფართო გამოყენების
არეალს, როგორც უსაფრთხოების, თავდაცვის და სამედიცინო, ასევე სამრეწველო
სფეროებში.

Ჩემი მთავარი მიზანი იყო შემექმნა ამ სისტემის მუშა პროტოტიპი, რომელიც
მარტივი ობიექტების, ადამიანების და სხვის ამოცნობას და მიზანში ამოღებას
შეძლებდა.

დაახლოებით 6 თვიანი შრომისგან მიღებული შედეგი არის ზემოხსენებული
სისტემის საპილოტე ვერსია, რომელსაც მანქანური სწავლების ალგორითმების
გამოყენებით შეუძლია მიკროკონტროლერით მართულ სისტემას კოორდინატები
გადასცეს, რომელიც შემდეგ End Effector-ს შესაბამისად მიმართავს.

შემდეგი ნაბიჯი, ცხადია, რომ არის პროექტის ზემოხსენებულ სფეროებში
განვრცობა და მათში გაჩენილი კონკრეტული პრობლემებისთვის მორგებაა.

# 3. Table of Contents: {#table-of-contents}

[2. ანოტაცია/Annotation [1](#ანოტაციაannotation)](#ანოტაციაannotation)

[3. Table of Contents: [5](#table-of-contents)](#table-of-contents)

[4. List of Formulas, Graphs, and Pictures
[6](#list-of-formulas-graphs-and-pictures)](#list-of-formulas-graphs-and-pictures)

[5. Introduction [7](#introduction)](#introduction)

[6. The Problem Statement & The Proposed Solution
[8](#the-problem-statement-the-proposed-solution)](#the-problem-statement-the-proposed-solution)

[6.1. Research Conducted Related to the Project Topic
[8](#research-conducted-related-to-the-project-topic)](#research-conducted-related-to-the-project-topic)

[6.2. The Technical Side of the Project
[10](#the-technical-side-of-the-project)](#the-technical-side-of-the-project)

[6.2.1. Mechanical Part [10](#mechanical-part)](#mechanical-part)

[6.2.2. Electronics & Control
[12](#electronics-control)](#electronics-control)

[6.2.2.1 Rotary Encoders & Angle Feedback
[13](#rotary-encoders-angle-feedback)](#rotary-encoders-angle-feedback)

[6.2.2.2 Motor Control [14](#motor-control)](#motor-control)

[6.2.2.3 PID Control [15](#pid-control)](#pid-control)

[6.2.2.4 Main Loop [18](#main-loop)](#main-loop)

[6.2.2.5 USART Communication
[18](#usart-communication)](#usart-communication)

[6.2.2.6 Camera Installation
[19](#camera-installation)](#camera-installation)

[6.2.3. Software [19](#software)](#software)

[6.2.3.1 The Main Program [20](#the-main-program)](#the-main-program)

[6.2.3.2 Machine Learning Model
[20](#machine-learning-model)](#machine-learning-model)

[6.3. Result & Comments [22](#result-comments)](#result-comments)

[7. Conclusion [23](#conclusion)](#conclusion)

[8. List of References & Resources Used
[25](#list-of-references-resources-used)](#list-of-references-resources-used)

[9. მადლობის გვერდი [26](#მადლობის-გვერდი)](#მადლობის-გვერდი)

[Appendix [28](#appendix)](#appendix)

# 4. List of Formulas, Graphs, and Pictures {#list-of-formulas-graphs-and-pictures}

**\#1 -** Initial Block Diagram (The end results moderately deviate from
this block diagram)  
**\#2 --** Gear Nomenclature  
**\#3 --** Design Process  
**\#4 --** STM32F103 Generic BluePill Board Pinout.  
**\#5 --** STM32 Encoder Mode Principle  
**\#6 -** STM32 PWM Formulas  
**\#7 -** PID Controller Gains Tuning Effects.  
**\#8 --** PID Controller Block Diagram  
**\#9 --** PID Controller Difference Equation after Bilinear Transform  
**\#10 --** Spirally wrapped cables for signal integrity  
**\#11 --** Screenshot: Machine learning model detecting tennis ball  
**\#12 --** Final look

# 5. Introduction {#introduction}

As I hadn\'t done any visually appealing projects during my time at
university, I wanted to work on a project that would soothe the Sci-Fi
starved mind of the person that grew up reading authors like Isaac
Asimov. The project would have to have moving parts that fit together in
an elegant way to look impressive to the outside observer, even one with
no engineering knowledge.

The project I chose to work on was the development of a target tracking
system that could detect and aim at a target object using a video
camera, a mechanical \"Pan-Tilt\" device, and a mounted end effector
(e.g., a laser). The system would be capable of tracking and maintaining
the target in its sights in real-time, showcasing the coordination of
various components in a visually appealing manner.

While the relevance and potential applications of this project have been
discussed in detail in the annotation, it is worth mentioning that such
a system could have diverse uses in fields like security, defense,
medical, and industrial automation, among others, depending on the
choice of the end effector and specific requirements.

**Note:**  
Sentences written using ~~strikethrough~~ represent previous versions of
the project that I choose to keep within the report.

# 6. The Problem Statement & The Proposed Solution {#the-problem-statement-the-proposed-solution}

## 6.1. Research Conducted Related to the Project Topic {#research-conducted-related-to-the-project-topic}

**Existing Technologies and Market in the Field:**

In recent years, there has been a growing interest in developing systems
that can autonomously track and maintain focus on a target object. Such
systems have found applications in various domains, including security
surveillance, defense tracking, industrial automation, and medical
procedures. The market for target tracking systems is projected to grow
significantly, driven by the increasing demand for autonomous solutions
and the integration of advanced technologies like computer vision and
robotics.

**Challenges in the Field:**

One of the major challenges in developing robust target tracking systems
is handling complex scenarios where the target may move erratically,
become occluded, or undergo changes in appearance. Traditional
approaches based on simple feature tracking or template matching often
fail in these situations. Additionally, integrating real-time tracking
with precise mechanical positioning and control mechanisms poses
significant technical challenges.

**Future Development Perspectives in the Field:**

The field of target tracking is rapidly evolving, with continuous
advancements in computer vision algorithms, machine learning techniques,
and robust control systems. The integration of deep learning models for
object detection and tracking is expected to greatly enhance the
performance and adaptability of these systems. Furthermore, the
development of low-cost and highly accurate positioning mechanisms, such
as Pan-Tilt devices, will enable the deployment of target tracking
solutions in a wider range of applications.

**Problem Statement and Novelty:**

This project aims to develop an innovative target tracking system that
can detect and maintain focus on a target object in real-time, using a
combination of computer vision techniques and a mechanical Pan-Tilt
mechanism with a mounted end effector (e.g., a laser). The system will
be capable of handling challenging scenarios, such as occlusions and
erratic movements, by leveraging advanced object detection and tracking
algorithms. The integration of the computer vision component with the
precise positioning mechanism represents a novel approach to creating a
visually appealing and practical target tracking solution.

**Technical Objectives and Requirements:**

- Develop a computer vision pipeline for robust object detection and
  tracking, capable of handling occlusions and appearance changes.

- Implement algorithms for accurate and smooth control of the Pan-Tilt
  mechanism, ensuring precise target positioning.

- Design and integrate a suitable end effector (e.g., laser pointer) to
  be mounted on the Pan-Tilt device.

- Create a user-friendly interface for target selection and system
  control.

**Relevance and Potential Applications:**

The developed target tracking system has the potential for applications
in various domains, such as security surveillance, defense tracking,
industrial automation (e.g., robotic assembly lines), and medical
procedures (e.g., surgical assistance). By demonstrating the seamless
integration of computer vision and mechanical positioning, this project
can pave the way for the development of more advanced and versatile
target tracking solutions.

**Commercialization Potential:**

The target tracking system developed in this project could have
significant commercial potential, particularly in industries where
precise and autonomous tracking is required. With further refinement and
customization for specific use cases, the system could be marketed as a
standalone product or integrated into existing solutions offered by
companies operating in the relevant domains.

## 6.2. The Technical Side of the Project {#the-technical-side-of-the-project}

As depicted in the initial block diagram modelled before the project
itself began, the discussion of the technical side of the project will
be split into 3 main and numerous sub-parts.

![](/media/imaged.png){width="6.0in" height="3.255022965879265in"}

*Picture \#1 -- Initial Block Diagram*

### 6.2.1. Mechanical Part {#mechanical-part}

The overall structure comprises three main components: a wooden base
housing the electronics, a pan mechanism, and a tilt mechanism. Each of
the pan and tilt mechanisms incorporates a DC motor, a rotary encoder, a
slip ring, and three 3D-printed helical gears for mechanical power
transmission.

![](/media/image12.png "Inserting image..."){width="4.158533464566929in"
height="2.5516808836395453in"}  
*Picture \#2 -- Gear Nomenclature*

- **The Wooden Base:** A box constructed from plywood with CNC-milled
  slots for the pan mechanism.

- **The Pan Mechanism:** The main platform is driven by a 2mm module
  140-tooth helical gear, rotated by a 27-tooth helical gear. Adjacent
  to the main gear, a rotary encoder is coupled with another 27-tooth
  helical gear. The slip ring is fed through the center of the main
  platform, allowing the electronics to pass through the box without
  tangling.

  - The rotary encoder has 200 pulses per rotation, but the STM32\'s
    encoder mode enables counting 800 pulses per rotation. With this
    gear ratio, each rotation of the main platform corresponds to
    approximately 4148 pulses (800 × 140/27), providing a precision of
    about 0.087 degrees.

- **The Tilt Mechanism:** Mounted onto the pan mechanism\'s main
  platform, the tilting platform is driven by a 1mm module 50-tooth
  double helical gear, rotated by a 25-tooth double helical gear.
  Adjacent to the main gear, the same rotary encoder is coupled with
  another 50-tooth double helical gear. The slip ring is fed through the
  side parallel to the gear.

  - As this uses the same rotary encoder with a 1:1 gear ratio, each
    rotation of the tilting platform corresponds to 800 pulses,
    providing a precision of 0.45 degrees.

The gear design was created using DS SolidWorks, which played a crucial
role in the initial design process and visualization. Significant time
was dedicated to learning and refining design skills, iterating through
various concepts for the project\'s appearance.

![](/media/image4.jpg){width="6.21875in"
height="3.3544728783902014in"}  
*Picture \#3 -- Design Process*

**Note:** The pan mechanism\'s gear ratio is not an exact 5:1 (140/28)
due to the failure to account for 3D printing tolerances when
manufacturing the gears. A 28-tooth version fit too tightly, posing a
risk of jamming the system. In practice, this small discrepancy did not
pose a significant problem.

**Note:** All moving parts are mounted with suitable ball bearings and
thrust bearings to minimize friction as much as possible.

### 6.2.2. Electronics & Control {#electronics-control}

The main controller is an STM32F103 Bluepill board handling motor
control, communication, and sensor inputs. The power source is a 12V
supply connected to a 220V AC outlet. A step-down converter provides 5V
to the Bluepill\'s 5V pin, and the on-board regulator supplies 3.3V to
the MCU. Each of the pan and tilt mechanisms have a 12Volt 300rpm
internal gearbox DC motor driven by a dual-channel motor driver, and a
200-pulse rotary encoder. The wooden enclosure has a power button with
an LED ring indicating power status, and two USB Type-B Female
connectors: one for connecting the tilt-mounted camera to a computer for
running the CV algorithm, and another for establishing a USART
connection between the computer and the STM32 to send and receive
information (e.g., new setpoint data).

![](/media/image13.png){width="3.118004155730534in"
height="2.187903543307087in"}  
*Picture \#4 -- STM32F103 Generic BluePill Board Pinout.*

#### 

#### 6.2.2.1 Rotary Encoders & Angle Feedback {#rotary-encoders-angle-feedback}

**Note:**  
Not used in the final version of the project, although still works and
could prove useful in future reiterations.

The rotary encoders are connected directly to the STM32 Bluepill board.
The STM32F103 has a High-Speed Timer (HRTIM) and three General Purpose
Timers (GPTIMs), each with 4 channels. Two GPTIMs are configured in
encoder mode, reserving two GPIO pins, one for each channel the encoder
mode requires. The A and B terminals of the encoders are connected to
these pins. In encoder mode, each pin, upon receiving a rising or
falling pulse (configured for both), triggers an interrupt that updates
the CNT Register of the timer based on which GPIO pin last fired the
interrupt, accounting for the spinning direction of the encoder shaft.
With this configuration counting both falling and rising pulses of both
A and B terminals of the encoder, the 200-pulse encoder effectively
becomes an 800-pulse encoder. Due to the 1:1 gear ratio, the tilt
mechanism\'s TIMx auto-reload value is set to 800, while the pan
mechanism, with a 27:140 ratio, has its TIMx auto-reload value set to
4148. These counter values, extracted from the TIMx-\>CNT Registers, can
be used to determine the angular position of the mechanism.

![](/media/image17.png){width="2.6562128171478565in"
height="1.3940944881889763in"}![](/media/image18.png){width="2.893079615048119in"
height="1.3861734470691163in"}  
*Picture \#5 -- STM32 Encoder Mode Principle*

For more details on the Encoder Mode:

<https://deepbluembedded.com/stm32-timer-encoder-mode-stm32-rotary-encoder-interfacing/>

#### 6.2.2.2 Motor Control {#motor-control}

The 12V DC Motors are driven by a DFRobot dual-channel motor driver
powered by a 12V power supply. Each channel has three control signals:
logical A and B signals for direction control, and a PWM signal for
speed control based on the duty cycle. The control signals come from
GPIO pins, and each channel has its own 3.3V power line from the MCU.
Two channels of the General Purpose Timer are used for PWM signals. The
MCU runs at 72MHz, and the PWM Resolution is set to 16 bits, giving a
PWM Frequency of approximately 1.1KHz (72Mhz/65535). The duty cycles of
the PWM signals are set using the TIMx-\>CCRx Registers. A PID
Controller is implemented on the STM32 to control the motors for
specific tasks.

![](/media/imagef.png){width="2.448380358705162in"
height="0.6370395888013998in"}![](/media/image10.png){width="2.838059930008749in"
height="0.641155949256343in"}

![](/media/image11.png){width="5.332255030621172in"
height="1.9944455380577428in"}  
*Picture \#6 - STM32 PWM Formulas*

For more details on PWM Control:

<https://deepbluembedded.com/stm32-pwm-example-timer-pwm-mode-tutorial/>

#### 6.2.2.3 PID Control {#pid-control}

The PID Controller for the system went through three iterations. Before
discussing them, let\'s first discuss the main outline of the
controller.

~~The setpoint is the target angle provided by the CV Algorithm on the
computer. The feedback term is the current angular position from the
encoder. Due to the wrap-around nature of the coordinates, the error
term is calculated as the minimum of the clockwise and counterclockwise
distances from the current position to the setpoint, allowing the
direction to be set outside the PID Controller. The error term in pulses
is scaled to a range of 0 to 180 degrees.~~

The setpoint is the center pixel of the frame provided by the mounted
camera, while the projected point is the center pixel of the largest
bounding box provided by the result of the machine learning model, which
is to be discussed in a later section.

The output is scaled to a number from --0.98 to 0.98 (I chose to keep
small overhead between the maximum and minimum inputs the controller can
take and the maximum and minimum outputs the PID controllers can
provide), with the sign representing direction, and the value
representing the percentage of the motor\'s maximum speed, then
multiplied by 65535 (the maximum duty cycle for a 16-bit PWM signal) and
written to the relevant CCRx register to set the motor speed. This
process constitutes one PID cycle.

The PID controller is written in C directly for the STM32, implemented
as a separate .h and .c file as a struct with its own methods/functions.
This allows creating multiple PID controllers with different gain
coefficients and output saturation limits. The controller used in the
project was tuned using trial and error, tweaking the gains in response
to the system\'s observed behavior, such as overshoot or insufficient
response time.

![](/media/image1b.png){width="5.22916447944007in"
height="2.214514435695538in"}  
*Picture \#7 - PID Controller Gains Tuning Effects.*

The first iteration of the PID controller was written with the most
basic PID arrangement, the P term proportional to the error, the I term
the sum of errors scaled by the gain, the D term the difference of the
current and last error.

![](/media/image1c.png){width="4.830435258092739in"
height="2.2248370516185476in"}  
*Picture \#8 - PID Controller Block Diagram*

The second iteration of the PID controller was written based on the
bilinear transform of the basic analog PID controller, implementing a
low pass filter with the D term to eliminate high frequency noise gain.

![](/media/image1d.png){width="5.14737532808399in"
height="3.0152307524059494in"}  
*Picture \#9 - PID Controller Difference Equation after Bilinear
Transform*

There were a couple of problems with these controllers, namely:

- Integrator windup

- Derivative "Kick" during the setpoint change

- Choosing sample time

The third and final iteration of the PID controller was implemented
using a library (See References) as a template. The sampling time for
scaling calculations is computed each cycle as the difference between
the current time and the last iteration, resulting in a dynamic sample
time. The derivative \"kick\" issue was resolved by calculating the
derivative term as the difference between the current and last
measurement, rather than the error, preventing sudden jumps due to
setpoint changes (Note: The sign of the D term changes due to this). To
address integrator windup, the integrator term is clamped to a minimum
and maximum value, which is sufficient for this application, although
other methods like dynamic integrator anti-windup exist.

I decided not to use the lowpass filter that is usually included with
the D term to keep the controller simple, instead, the lowpass filter is
integrated into the python script that generates the input coordinates
(projected points) to the system via moving average.

~~While the library simplified the implementation, it did not account
for the wrap-around nature of the system. Modifications were made to the
error and dInput (dInput = input - uPID-\>LastInput) calculations within
the library to address this. The change involves taking the minimum of
the counterclockwise and clockwise differences for each calculation,
ensuring that there are no sudden jumps in the difference. For example,
when the main platform passes 4148 pulses and wraps around to 0, the
dInput should not experience a large jump.~~

#### 6.2.2.4 Main Loop {#main-loop}

The main program starts by initializing the two PID controllers for the
pan and tilt mechanisms, followed by initializing all the MCU
peripherals using the generated code from the STM32CubeIDE. The main
while loop consists of back-and-forth USART communication and a call to
the \"runMotors(&basePID, &tiltPID, base_proj_point, tilt_proj_point,
base_set_point, tilt_set_point)\" method, which updates the motor speed
based on the given parameters and the PID Controllers passed.

The main program has two main changing variables, base_proj_point and
tilt_proj_point, which remain equal to the constant setpoint (the center
pixel of the camera frame) unless updated from the serial data received
by the STM32 from the computer. The runMotors() method runs the PID
algorithm for both motors each cycle. In steady state, when the setpoint
hasn\'t changed, the system is expected to remain still.

#### 6.2.2.5 USART Communication {#usart-communication}

The communication between the MCU and the computer is established using
the USB2TTL Unit connected to the Bluepill board. The USB2TTL chip
allows sending serial data back and forth.

~~The HAL_UART_Receive_IT() function generates an interrupt each time a
character is received in the input stream. To receive both coordinates
at once, the USART interrupt routine was modified as follows: During
each ISR, the global variables baseBuffer or tiltBuffer are updated. The
\'a\' character is arbitrarily selected as the sentinel character.
Initially, the baseBuffer is updated until the sentinel character
appears, after which the following numbers update the tiltBuffer. After
the second \'a\' character, the contents of both buffers are moved into
baseSetPoint and tiltSetPoint, which are then fed into the runMotors()
method to drive the motors.~~

The reception of data is accomplished using Direct Memory Access (DMA).
This approach is superior to the previous interrupt-based approach
because the HAL_UART_Receive_DMA() function receives bytes into its
buffer without relying on the CPU to execute the interrupt service
routine (ISR). This means that the CPU has a reduced workload, allowing
it to focus on other tasks.

Each time the buffer is filled, a callback function is invoked, which
parses the received bytes into numbers and updates the relevant
variables in the code. The DMA is configured to use a 6-byte buffer,
expecting the data in the following format: \"xxxyyy\", where \'xxx\'
represents the X coordinate of the object, and \'yyy\' represents the Y
coordinate.

Additionally, a simple synchronization mechanism is implemented to
ensure that the data is not received out of phase (e.g., \"xxyyyx\").
This mechanism works by first arming the DMA with a 1-byte buffer,
waiting for a start character (in this case, \'a\'), and then rearming
the DMA with a 6-byte buffer to receive the 6 relevant bytes. This
transforms the data format into \"axxxyyy\", where \'a\' is the starting
character.

Independent from what was previously discussed, the MCU regularly sends
the important bits of data to the computer.

#### 6.2.2.6 Camera Installation {#camera-installation}

The camera used in the project is a generic USB camera mounted on the
tilt mechanism. The camera\'s cable is fed through two sets of slip
rings and routed out of the enclosure using a USB A-to-USB B adapter
that passes through the wall of the enclosure.

One of the challenges faced during the implementation was the
degradation of signal integrity caused by the slip rings and the process
of cutting and resoldering the wires. This issue prevented the laptop
from recognizing the USB camera as a valid device.

To resolve this problem, the following solution was implemented: the
data+ and data- wires were spirally wrapped around each other, and all
solder was removed from the wires. Instead of soldering, the Western
Union splice method was used to combine the wires. This approach helped
restore the signal integrity sufficiently for the camera to be
recognized as a USB device and function correctly.

![](/media/image1e.png){width="2.256369203849519in"
height="1.9187587489063866in"}  
*Picture \#10 -- Spirally wrapped cables for signal integrity*

### 6.2.3. Software {#software}

The software is built in the Python programming language, utilizing the
OpenCV and Ultralytics libraries for handling machine learning-related
tasks. The script establishes a serial connection between itself and the
STM32 microcontroller and accesses the mounted camera.

#### 6.2.3.1 The Main Program {#the-main-program}

The camera captures frames and sends them in grayscale to the Python
script, which processes these frames through a machine learning model
trained to detect a specific object (in this case, tennis balls). The
script then plots bounding boxes onto the frames, filtering out boxes
with a confidence score below 0.5. Subsequently, it calculates the
center coordinate of the largest remaining bounding box and transmits
this data over the serial connection.

To improve the stability of the transmitted data, a moving average
filter is applied to smooth the calculated coordinates before sending
them to the microcontroller, although this moving average introduces a
delay into the system, so window size can vary depending on the delay
acceptable for the system.

#### 6.2.3.2 Machine Learning Model {#machine-learning-model}

The project utilizes the YOLOv8n model from Ultralytics for object
detection. A pre-trained YOLOv8n model was fine-tuned using open-source
datasets obtained from Roboflow. The training process was carried out on
Google Cloud GPUs using Google Colab.

~~Since the camera used in the project captures grayscale images, all
the training data was preprocessed into grayscale format. Additionally,
the training data was carefully curated to include the target object
(tennis ball) at various sizes, ensuring that the model can detect the
object accurately, both at closer and farther distances.~~

The original grayscale camera was replaced with a high-resolution RGB
camera capable of 30 frames per second (the frames per second slightly
limit the speed of the object that can still be recognized by the
model). The model was trained on a diverse dataset of color images
featuring tennis balls at various sizes, enabling object detection
across different distances. Running on an RTX3050 GPU with CUDA
acceleration, the model\'s inference time was significantly reduced from
90ms (on CPU) to approximately 7ms.

A key challenge in using less powerful hardware for this task is the
latency in transmitting projected point information. This delay hinders
the PID controller\'s ability to react promptly, necessitating a
compromise on system speed. Conversely, with reduced latency, the system
demonstrates markedly improved accuracy and responsiveness.

The decision to use a tennis ball as the target object was made due to
the abundance of labeled data available online and the ease of acquiring
tennis balls for testing and demonstration purposes.

![](/media/image1f.png){width="6.739584426946632in"
height="5.41666447944007in"}  
*Picture \#11 -- Screenshot: Machine learning model detecting tennis
ball*

## 6.3. Result & Comments {#result-comments}

The resulting aiming system from this project operates with a slight
delay, which can be attributed to both hardware limitations of the
camera and the laptop running the object detection algorithm, as well as
non-optimal code implementation. However, this delay could have been
mitigated by utilizing a better-trained machine learning model, leaving
room for improvement.

Regarding the motor control, while the current PID coefficient
configuration works well enough for demonstration purposes, further
tuning could potentially enhance the overall performance.

A significant limitation arises from the combination of a poorly trained
machine learning model and a low frame rate camera. This setup struggles
to recognize rapidly moving objects. When the system loses sight of the
target, it defaults to the center of the frame for the projected point.
This behavior results in abrupt jumps in the projected point,
effectively introducing high-frequency noise.

This noise poses a challenge for the derivative (D) term in a PID
controller. Without a robust low-pass filter---which would introduce its
own delay---the D term becomes nearly unusable. To address this, I opted
to implement a simple PI controller instead, effectively setting the D
coefficient to zero or a value very close to it. This adjustment proved
adequate for maintaining acceptable system performance.

The project has largely achieved its initial goal of creating a
real-time tracking system, albeit with a minor delay. The current
performance closely approximates the original objective, and there\'s
significant potential for improvement. Through further fine-tuning, code
optimization, and modest hardware upgrades, the system could likely meet
or exceed the initial performance targets.

The primary constraint is hardware limitations, particularly given the
computational demands of computer vision tasks. These operations require
a substantial number of calculations per second, making hardware
capabilities a critical factor in system performance.

It\'s important to note that this project showcases the challenges and
limitations that can arise when working with real-world hardware and
software integration, particularly in the realm of computer vision and
robotics. Despite not achieving real-time performance, the project
demonstrates the ability to implement object detection, data
transmission, and motor control effectively, providing a solid
foundation for further improvements and refinements.

![](/media/image20.png){width="5.0625in" height="6.739584426946632in"}

*Picture \#12 -- Final Look*

#  7. Conclusion {#conclusion}

The target tracking system developed in this project successfully
demonstrates the integration of computer vision techniques and
mechanical positioning mechanisms to achieve real-time object tracking
and aiming. The system utilizes a custom-built mechanical assembly
featuring a Pan-Tilt device driven by DC motors and controlled by an
STM32 microcontroller. The computer vision component leverages the
YOLOv8n object detection model fine-tuned on a dataset of tennis balls,
enabling robust detection and tracking of the target object.

The project\'s results showcase the system\'s ability to detect and
maintain focus on a target object, albeit with a slight delay caused by
hardware limitations and non-optimal code implementation. The use of a
moving average filter introduces additional latency but improves the
stability of the transmitted data. While the system falls slightly short
of achieving true real-time performance, the project highlights the
potential for further improvements through hardware upgrades, code
optimization, and better model training.

Throughout the development process, several interesting challenges were
encountered and overcome. Preserving the signal integrity of the USB
camera cable while modifying it extensively by feeding it through slip
rings and cutting and recombining it was a significant hurdle. Ensuring
synchronized and correct data transfer over the serial connection
required careful implementation. Diving deep into digital PID systems,
understanding tuning approaches, implementation details, and addressing
issues like integrator windup and derivative kick expanded the knowledge
gained.

Additionally, learning to account for material expansion during 3D
printing and designing mechanical parts that fit together precisely was
a valuable experience. Mastering efficient data reception techniques on
the STM32 and refining C programming skills were essential aspects of
the project. Furthermore, understanding deep learning algorithms,
preventing overfitting, leveraging online resources for data science
tasks requiring high computing power, and getting a grasp of object
detection techniques in deep learning were crucial components of the
project\'s success.

Future plans for this project may involve exploring more advanced object
detection and tracking algorithms, implementing more sophisticated
control strategies, and optimizing the system\'s performance for
specific use cases. Additionally, the integration of alternative end
effectors, such as robotic arms or specialized tools, could expand the
system\'s capabilities and potential applications.

Overall, this project serves as a compelling demonstration of the
synergy between computer vision, robotics, and mechanical engineering,
offering a solid foundation for further research and development in the
field of autonomous target tracking systems.

# 8. List of References & Resources Used {#list-of-references-resources-used}

[https://www.youtube.com/watch?v=ZhDO16FDmxA  
](https://www.youtube.com/watch?v=ZhDO16FDmxA)[https://www.youtube.com/watch?v=uNZmLPZHon8&t=23s  
](https://www.youtube.com/watch?v=uNZmLPZHon8&t=23s)[https://www.youtube.com/watch?v=zOByx3Izf5U  
](https://www.youtube.com/watch?v=zOByx3Izf5U)[https://www.youtube.com/playlist?list=PLrOFa8sDv6jcp8E3ayUFZ4iNI8uuPjXHe  
](https://www.youtube.com/playlist?list=PLrOFa8sDv6jcp8E3ayUFZ4iNI8uuPjXHe)[https://www.youtube.com/playlist?list=PLS1QulWo1RIa7D1O6skqDQ-JZ1GGHKK-K  
](https://www.youtube.com/playlist?list=PLS1QulWo1RIa7D1O6skqDQ-JZ1GGHKK-K)[https://www.youtube.com/playlist?list=PLNyfXcjhOAwO5HNTKpZPsqBhelLF2rWQx  
](https://www.youtube.com/playlist?list=PLNyfXcjhOAwO5HNTKpZPsqBhelLF2rWQx)[https://deepbluembedded.com/stm32-arm-programming-tutorials/  
](https://deepbluembedded.com/stm32-arm-programming-tutorials/)[https://github.com/Majid-Derhambakhsh/PID-Library  
](https://github.com/Majid-Derhambakhsh/PID-Library)[https://eprints.gla.ac.uk/3815/1/IEEE_CS_PID_01580152.pdf  
](https://eprints.gla.ac.uk/3815/1/IEEE_CS_PID_01580152.pdf)[https://www.sciencedirect.com/science/article/pii/S1474667015386900  
](https://www.sciencedirect.com/science/article/pii/S1474667015386900)[https://universe.roboflow.com/  
](https://universe.roboflow.com/)<https://colab.research.google.com/>

# 9. მადლობის გვერდი {#მადლობის-გვერდი}

მინდა მადლობა გადავუხადო შემდეგ ადამიანებს:

- **ზვიად სულაბერიძეს** - რიგ საკითხებში დახმარების გაწევისთვის.

- **ირაკლი შელიას** - პრეზენტაციაზე გამოყენებული ლეპტოპის თხოვებისთვის,
  რომლემაც დიდ წილად განაპირობა ის, რომ პროექტი მუშაობს.

- **ნიკა მიქაბერიძეს და ლუკა დარსალიას** - მანქანურ სწავლებასთან
  დაკავშირებულ საკითხებში დახმარებისთვის.

- **ირაკლი დონგუზაშვილს** - ლაბორატორიაში არსებული ინვენტარის 90%-ის
  წყაროობისთვის.

- **ვაჟა ფურცელაძეს** - რიგ საკითხებში დახმარების გაწევისთვის.

- **გუგა ვარდიაშვილს** - ჩვენი კურსის გვერდში დგომისთვის.

- **მთლიან ECE21-ს** - იმისთვის, რომ საუკეთესოები არიან.

- **ავთო მღებრიშვილს** - პროექტის დასაწყისში მისი პრინტერის თხოვნისთვის.

# 

# 

# 

# Appendix

<https://github.com/AlwaysAbia/ECE21_Junior_Project>
