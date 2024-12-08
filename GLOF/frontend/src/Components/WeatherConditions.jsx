import { useState } from 'react';
import axios from 'axios';
import { Link } from 'react-router-dom';
import { Line } from 'react-chartjs-2';
import { Chart as ChartJS, CategoryScale, LinearScale, PointElement, LineElement, Title, Tooltip, Legend } from 'chart.js';

ChartJS.register(CategoryScale, LinearScale, PointElement, LineElement, Title, Tooltip, Legend);

const WeatherConditions = () => {
    const [weatherData, setWeatherData] = useState(null);
    const [place, setPlace] = useState("")
    const [error, setError] = useState(null);

    const handleWeatherData = async () => {
        try {
            console.log(place)
            const resp = await axios.get(`https://glacial-lake-outburst-floods-system.onrender.com/api/weatherData/${place}`);
            setWeatherData(resp.data);
            setError(null); // Clear any previous errors
        } catch (err) {
            console.log("Error fetching weather data: ", err);
            setError("Failed to fetch weather data. Please try again later.");
        }
    };

    const chartData = {
        labels: ['Temperature', 'Humidity'],
        datasets: [
            {
                label: 'Current Weather',
                data: [
                    weatherData?.current?.temp_c || 0,
                    weatherData?.current?.humidity || 0
                ],
                backgroundColor: 'rgba(75, 192, 192, 0.2)',
                borderColor: 'rgba(75, 192, 192, 1)',
                borderWidth: 1,
            },
        ],
    };


    return (
        <div>
            <input className='border-2 border-black py-2 px-4 m-4 bg-slate-400 rounded-md' type="text" value={place} onChange={(e) => setPlace(e.target.value)} />
            <button className='border py-2 px-4 border-black rounded-lg bg-slate-900 text-slate-200 mx-2' onClick={handleWeatherData}>Submit</button>
            <Link to="/airQuality"><button className='border py-2 px-4 border-black rounded-lg bg-slate-900 text-slate-200 mx-2'>Air Quality</button></Link>
            <Link to="/temp-humidity"><button className='border py-2 px-4 border-black rounded-lg bg-slate-900 text-slate-200 mx-2' >Temp and humidity</button></Link>
            {error && <p>{error}</p>}
            {weatherData && (
                <div>
                    <h3>Weather Conditions:</h3>

                    <p>Place Name: {weatherData.location.name}</p>
                    <p>Region: {weatherData.location.region}</p>
                    <p>Country: {weatherData.location.country}</p>
                    <p>Temperature: {weatherData.current.temp_c}°C</p>
                    <p>Condition: {weatherData.current.condition.text}</p>
                    <p>Wind Speed in MPH: {weatherData.current.wind_mph}</p>
                    <div className='w-[500px] h-[500px]'>
                        <Line data={chartData} />
                    </div>
                </div>
            )}
        </div>
    );
};

export default WeatherConditions;
