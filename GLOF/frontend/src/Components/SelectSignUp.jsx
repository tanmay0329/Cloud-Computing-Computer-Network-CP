
import { Link } from 'react-router-dom'
const SelectSignUp = () => {
  return (
    <div className='flex flex-col justify-center items-center bg-slate-900 font-light  h-screen'>
      <h1 className='mb-5 text-white text-3xl'>Sign Up</h1>
      <div className='mb-10 text-white text-3xl'>as</div>
      <div className='flex flex-col'>
      <Link to="/genral-public-sign-up"><button className='my-2 text-xl bg-slate-700 rounded-md text-white py-2 px-4 hover:shadow-md hover:shadow-slate-300 transition-all duration-300 hover:-translate-y-1 w-full'>General Public</button></Link>
      <Link to="/government-auth-sign-up"><button className='my-2 text-xl bg-slate-700 rounded-md text-white py-2 px-4 hover:shadow-md hover:shadow-slate-300 transition-all duration-300 hover:-translate-y-1'>Government Authorities</button></Link>
      <Link to="/emergency-resp-sign-up"><button className='my-2 text-xl bg-slate-700 rounded-md text-white py-2 px-4 hover:shadow-md hover:shadow-slate-300 transition-all duration-300 hover:-translate-y-1'>Emergency Responders</button></Link>
      </div>
    </div>
  )
}

export default SelectSignUp
