
import { Link } from 'react-router-dom'
import glofImg from "../assets/glof-img.jpg"

const Home = () => {
  return (
    <div>
    <div className='bg-slate-900 text-white '>
      <div className='flex justify-between bg-slate-800 py-4 px-2'>
        <span className='text-5xl '>GLOFMAS</span>
        <span>
          <span className='font-light text-xl mx-4'>About Us</span>
          <Link to="contact-us"><span className='font-light text-xl mx-4'>Contact Us</span></Link>
          <Link to="/select-sign-up"><span className='font-light text-xl mx-4'>Sign Up</span></Link>
        </span>
      </div>
      <div className='flex flex-col justify-center items-center '>
        <h1 className='font-light text-4xl my-5'>Glacial Lake Outburst Floods </h1>
        <h1 className='font-light text-3xl '>Monitoring and Alert System </h1>
        <Link to="/knowmore"><button className='py-2 px-4 bg-slate-600 rounded-xl my-10 mx-10 mt-20 hover:shadow-white hover:-translate-y-1 duration-300 shadow-md'>Know More</button></Link>
      </div>
    </div>
    <div className='flex flex-col  h-screen '>
      <img src={glofImg} alt="" className='-z-10 absolute opacity-80'/>
      <div className='bg-slate-300 rounded-3xl bg-opacity-50 text-3xl text-center text-black my-10 font-light mx-10 py-10'>Glacial Lake Outburst Floods (GLOFs) pose significant risks to communities living downstream of glacial lakes. Our system is designed to monitor these lakes, predict potential outburst events, and provide timely alerts to minimize risks and enhance preparedness.</div>
      <div className='bg-slate-300 rounded-3xl bg-opacity-50 text-3xl text-center text-black my-10 font-light mx-10 py-10'>What is a GLOF?
      GLOFs occur when the natural barriers containing glacial lakes fail, releasing large volumes of water suddenly. These floods can cause widespread damage, including loss of life, property, and infrastructure. Understanding the risks and staying informed are critical to protecting lives and livelihoods.</div>
    </div>
  </div>
  )
}

export default Home
