// import React from 'react'
import { Link } from 'react-router-dom'

const Header = () => {
  return (
    <div className='flex justify-between bg-slate-800 py-4 px-2 text-white'>
          <span className='text-2xl '>GLOFMAS</span>
          <span>
            <span className='font-light text-xl mx-4'>About Us</span>
            <Link to="contact-us"><span className='font-light text-xl mx-4'>Contact Us</span></Link>
            <Link to="/main"><span className='font-light text-xl mx-4'>Home</span></Link>
            <Link to="/"><span className='font-light text-xl mx-4'>Main Page</span></Link>
          </span>
        </div>
  )
}

export default Header
