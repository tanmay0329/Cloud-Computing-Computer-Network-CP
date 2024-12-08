
import { Link } from 'react-router-dom'
import vidGlof from "../assets/glof-demo.mp4"

import newsImg1 from "../assets/newsImg-1.avif"
import newsLogo1 from "../assets/newslogo-1.avif"

import newsImg2 from "../assets/newsImg-2.jpeg"
import newsLogo2 from "../assets/newslogo-2.svg"

import newsImg3 from "../assets/newsImg-3.webp"
import newsLogo3 from "../assets/newslogo-3.svg"



const KnowMore = () => {
    return (

        <div>
            <div className='bg-slate-900 text-white '>
                <div className='flex justify-between bg-slate-800 py-4 px-2'>
                    <span className='text-3xl '>GLOFMAS</span>
                    <span>
                        <span className='font-light text-xl mx-4'>About Us</span>
                        <Link to="contact-us"><span className='font-light text-xl mx-4'>Contact Us</span></Link>
                        <Link to="/select-sign-up"><span className='font-light text-xl mx-4'>Sign Up</span></Link>
                    </span>
                </div>
                <div className='flex flex-col justify-center items-center '>
                    <h1 className='font-light text-4xl my-5 mb-20'> Know More About   Glacial Lake Outburst Floods </h1>

                </div>
                <div className='flex flex-col justify-center items-center'>
                    <h2 className=' mx-5 font-light text-2xl my-5'>Video Demo of How it Occures</h2>
                    <video className='w-[800px] border-2 border-white mx-10 ' controls>
                        <source src={vidGlof} type="video/mp4" />
                    </video>
                </div>
                <div className='flex flex-col justify-center items-center '>
                    <h2 className=' mx-5 font-light text-2xl my-5 mt-20'>News About GLOF</h2>
                    <div className='my-5 w-[800px] bg-slate-200 rounded-lg hover:shadow-slate-500 shadow-md duration-300 hover:-translate-y-2 text-slate-900'>
                        <img src={newsLogo1} className='w-28  mx-4 my-2' alt="" />
                        <div className='mx-2 my-1 text-xl font-bold'>Rising threats in the Himalayas: Glacial lake outburst floods</div>
                        <img src={newsImg1} className='w-40 rounded-md mx-4 my-2' alt="" />
                        <div className='mx-2 my-1'>On the intervening night of October 3-4, 2023, South Lhonak, a glacial lake in the Sikkim Himalaya burst,
                            causing flood and huge destruction downstream. More than 150 people reportedly died,
                            and critical infrastructure, including the Teesta-III dam and agricultural land, were
                            damaged in the devastating event.</div>
                        <div className='mx-2 my-4  underline text-blue-700'>

                            <a className='underline ' href="https://www.newindianexpress.com/xplore/2024/Sep/18/rising-threats-in-the-himalayas-glacial-lake-outburst-floods">Full Article</a>
                        </div>
                    </div>
                    <div className='my-5 w-[800px] bg-slate-200 rounded-lg hover:shadow-slate-500 shadow-md duration-300 hover:-translate-y-2 text-slate-900'>
                        <img src={newsLogo2} className='w-28  mx-4 my-2' alt="" />
                        <div className='mx-2 my-1 text-xl font-bold'>Government to review design of dams vulnerable to glacial lake outburst floods</div>
                        <img src={newsImg2} className='w-40 rounded-md mx-4 my-2' alt="" />
                        <div className='mx-2 my-1'>The government on Thursday (August 8, 2024) said it will review the design of all existing and under-construction
                             dams vulnerable to Glacial Lake Outburst Floods, ensuring these dams have adequate 
                             spillway capacity to handle extreme flood scenarios..</div>
                        <div className='mx-2 my-4  underline text-blue-700'>

                            <a className='underline ' href="https://www.thehindu.com/sci-tech/energy-and-environment/government-to-review-design-of-dams-vulnerable-to-glacial-lake-outburst-floods/article68500707.ece">Full Article</a>
                        </div>
                    </div>

                    <div className='my-5 w-[800px] bg-slate-200 rounded-lg hover:shadow-slate-500 shadow-md duration-300 hover:-translate-y-2 text-slate-900'>
                        <img src={newsLogo3} className='w-28  mx-4 my-2' alt="" />
                        <div className='mx-2 my-1 text-xl font-bold'>Govt launches programme to tackle glacial lake outburst flood threat in Himalayas, 
                            all 190 high-risk lakes to be monitored</div>
                        <img src={newsImg3} className='w-40 rounded-md mx-4 my-2' alt="" />
                        <div className='mx-2 my-1'>NEW DELHI: The glacial lake outburst flood (GLOF) in Sikkim , which occurred in Oct 2023 and claimed 14 lives,
                             has highlighted the vulnerability of the entire Himalayan region to such catastrophic
                              events. This tragedy prompted the Centre to launch a comprehensive national  GLOF  
                              risk mitigation programme, targeting 190 high-risk glacial lakes  across four states 
                              and two Union Territories.</div>
                        <div className='mx-2 my-4  underline text-blue-700'>

                            <a className='underline ' href="https://www.thehindu.com/sci-tech/energy-and-environment/government-to-review-design-of-dams-vulnerable-to-glacial-lake-outburst-floods/article68500707.ece">Full Article</a>
                        </div>
                    </div>

                </div>
            </div>
        </div>
    )
}

export default KnowMore
